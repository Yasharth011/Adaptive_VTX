"""
Closed-Loop Adaptive VTX System — OpenCV Video Quality Analyzer
================================================================
Extracts RF link quality metrics from the live analog video feed
captured via the Skydroid receiver's USB video output (or a USB
video capture card connected to the AV-OUT).

How it works
────────────
Analog FPV video degrades in characteristic ways as RF link weakens:

  STRONG SIGNAL  →  Clean, sharp frames, low noise, stable brightness
  MARGINAL LINK  →  Salt-and-pepper noise, horizontal scan artifacts,
                    reduced contrast, chroma smearing
  WEAK / LOST    →  Full-frame noise ("snow"), black frames, rolling
                    sync, complete dropout

We quantify these effects per-frame using:

  1. Noise Level      — variance of Laplacian (high = noisy/blurry)
  2. Contrast         — RMS contrast of luma channel
  3. Temporal Delta   — mean absolute difference between frames
                        (high = flickering noise, not real motion)
  4. Blocking Artifacts — DCT block-boundary energy (analog noise causes
                          pseudo-blocking in compressed captures)
  5. Dropout Ratio    — fraction of pixels near pure black or pure white
                        (clipped pixels = signal loss)
  6. SSIM proxy       — structural similarity vs. a short rolling clean
                        reference (drops when link degrades)
  7. Sync Loss Flag   — detected via sudden luminance histogram collapse

These 7 features feed a weighted scorer → RSSI proxy (dBm) + SNR (dB).

Capture setup
─────────────
Option A: USB video capture card (recommended)
  Skydroid AV-OUT (RCA yellow) → USB capture card → laptop
  Device appears as /dev/video0 or DirectShow "USB Video"

Option B: Skydroid USB direct (if it exposes a UVC video device)
  Plug Skydroid USB → laptop, check if it appears as a camera

Pipeline (complete, no middleman)
──────────────────────────────────
  RunCam → VTX → 5.8GHz RF → Skydroid RX → USB capture card
                                                    ↓
                                        video_analyzer_cv.py
                                         (OpenCV 7 metrics)
                                                    ↓  WiFi UDP
                                               ESP32-S3
                                           (SmartAudio v2)
                                                    ↓
                                         AKK Race Ranger VTX

Dependencies
────────────
  pip install opencv-python numpy scipy rich

Usage
─────
  python analyzer.py --device 0 --dashboard --show-preview
  python analyzer.py --device 0 --telem 192.168.4.1 --dashboard
  python analyzer.py --device 0 --telem 192.168.4.1 --log flight.csv
"""

import argparse
import csv
import json
import logging
import sys
import time
from collections import deque
from dataclasses import dataclass, asdict
from datetime import datetime
from typing import Optional

import cv2
import numpy as np
import socket
from scipy.ndimage import uniform_filter
from rich.console import Console
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich import box

# ─── Logging ──────────────────────────────────────────────────────────────────
logging.basicConfig(
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
    level=logging.INFO,
)
log = logging.getLogger("cv_analyzer")

# ─── Tunable Parameters ───────────────────────────────────────────────────────

RSSI_CLEAN_DBM = -55.0
RSSI_NOISY_DBM = -95.0

W_NOISE    = 0.30
W_CONTRAST = 0.15
W_TEMPORAL = 0.20
W_DROPOUT  = 0.20
W_BLOCKING = 0.05
W_SSIM     = 0.10

NOISE_CLEAN    = 50.0
NOISE_BAD      = 800.0
CONTRAST_MIN   = 20.0
CONTRAST_MAX   = 80.0
TEMPORAL_CLEAN = 5.0
TEMPORAL_NOISY = 60.0
DROPOUT_MAX    = 0.15

SSIM_REF_LEN = 10
HISTORY      = 60


# ─── Metrics Dataclass ────────────────────────────────────────────────────────
@dataclass
class FrameMetrics:
    timestamp:      float
    frame_index:    int
    noise_level:    float
    contrast:       float
    temporal_delta: float
    dropout_ratio:  float
    blocking_score: float
    ssim_score:     float
    signal_quality: float
    rssi_proxy_dbm: float
    snr_db:         float
    sync_lost:      bool
    rssi_avg:       float
    snr_avg:        float
    quality_avg:    float


# ─── Frame Feature Extractor ──────────────────────────────────────────────────
class FrameAnalyzer:
    def __init__(self):
        self._prev_gray:      Optional[np.ndarray] = None
        self._ref_frames:     deque = deque(maxlen=SSIM_REF_LEN)
        self._best_ssim_ref:  Optional[np.ndarray] = None
        self._frame_idx       = 0
        self._rssi_hist       = deque(maxlen=HISTORY)
        self._snr_hist        = deque(maxlen=HISTORY)
        self._quality_hist    = deque(maxlen=HISTORY)

    def _noise_level(self, gray: np.ndarray) -> float:
        lap = cv2.Laplacian(gray, cv2.CV_64F)
        return float(lap.var())

    def _contrast(self, gray: np.ndarray) -> float:
        return float(gray.std())

    def _temporal_delta(self, gray: np.ndarray) -> float:
        if self._prev_gray is None:
            return 0.0
        return float(cv2.absdiff(gray, self._prev_gray).mean())

    def _dropout_ratio(self, gray: np.ndarray) -> float:
        black = np.sum(gray < 5)
        white = np.sum(gray > 250)
        return float((black + white) / gray.size)

    def _blocking_score(self, gray: np.ndarray) -> float:
        h, w = gray.shape
        f = gray.astype(np.float32)
        h_diffs = [np.abs(f[r, :] - f[r-1, :]).mean() for r in range(8, h-8, 8)]
        v_diffs = [np.abs(f[:, c] - f[:, c-1]).mean() for c in range(8, w-8, 8)]
        if not h_diffs and not v_diffs:
            return 0.0
        return float(np.mean(h_diffs + v_diffs))

    def _ssim_proxy(self, gray: np.ndarray) -> float:
        if self._best_ssim_ref is None:
            return 1.0
        ref = self._best_ssim_ref
        if ref.shape != gray.shape:
            ref = cv2.resize(ref, (gray.shape[1], gray.shape[0]))
        scale = 0.25
        sz = (max(1, int(gray.shape[1] * scale)), max(1, int(gray.shape[0] * scale)))
        a  = cv2.resize(gray, sz).astype(np.float64)
        b  = cv2.resize(ref,  sz).astype(np.float64)
        c1, c2 = 6.5025, 58.5225
        mu_a = uniform_filter(a, 11)
        mu_b = uniform_filter(b, 11)
        mu_aa = uniform_filter(a * a, 11) - mu_a ** 2
        mu_bb = uniform_filter(b * b, 11) - mu_b ** 2
        mu_ab = uniform_filter(a * b, 11) - mu_a * mu_b
        ssim_map = ((2 * mu_a * mu_b + c1) * (2 * mu_ab + c2)) / \
                   ((mu_a**2 + mu_b**2 + c1) * (mu_aa + mu_bb + c2))
        return float(np.clip(ssim_map.mean(), 0.0, 1.0))

    def _update_reference(self, gray: np.ndarray, noise: float, ssim: float):
        self._ref_frames.append((noise, gray.copy()))
        self._best_ssim_ref = min(self._ref_frames, key=lambda x: x[0])[1]

    def _sync_lost(self, gray: np.ndarray, noise: float, dropout: float) -> bool:
        hist = cv2.calcHist([gray], [0], None, [8], [0, 256]).flatten()
        hist = hist / hist.sum()
        collapsed = (hist[0] + hist[-1]) > 0.80
        snowy = noise > NOISE_BAD * 1.5
        return bool(collapsed or (snowy and dropout > 0.40))

    def _score_and_map(self, noise, contrast, temporal, dropout, blocking, ssim) -> tuple:
        n_noise    = float(np.clip(1.0 - (noise - NOISE_CLEAN) / max(NOISE_BAD - NOISE_CLEAN, 1), 0, 1))
        n_contrast = float(np.clip((contrast - CONTRAST_MIN) / max(CONTRAST_MAX - CONTRAST_MIN, 1), 0, 1))
        n_temporal = float(np.clip(1.0 - (temporal - TEMPORAL_CLEAN) / max(TEMPORAL_NOISY - TEMPORAL_CLEAN, 1), 0, 1))
        n_dropout  = float(np.clip(1.0 - dropout / max(DROPOUT_MAX, 1e-3), 0, 1))
        n_blocking = float(np.clip(1.0 - blocking / 20.0, 0, 1))
        n_ssim     = float(np.clip(ssim, 0, 1))
        quality    = (W_NOISE * n_noise + W_CONTRAST * n_contrast + W_TEMPORAL * n_temporal +
                      W_DROPOUT * n_dropout + W_BLOCKING * n_blocking + W_SSIM * n_ssim) * 100.0
        quality    = float(np.clip(quality, 0.0, 100.0))
        t          = quality / 100.0
        rssi       = RSSI_NOISY_DBM + t * (RSSI_CLEAN_DBM - RSSI_NOISY_DBM)
        snr        = float(np.clip(quality * 0.40, 0.0, 40.0))
        return quality, round(rssi, 2), round(snr, 2)

    def process(self, frame: np.ndarray) -> FrameMetrics:
        small     = cv2.resize(frame, (320, 240))
        gray      = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
        noise     = self._noise_level(gray)
        contrast  = self._contrast(gray)
        temporal  = self._temporal_delta(gray)
        dropout   = self._dropout_ratio(gray)
        blocking  = self._blocking_score(gray)
        ssim      = self._ssim_proxy(gray)
        sync_lost = self._sync_lost(gray, noise, dropout)
        quality, rssi, snr = self._score_and_map(noise, contrast, temporal, dropout, blocking, ssim)
        self._update_reference(gray, noise, ssim)
        self._prev_gray = gray
        self._frame_idx += 1
        self._rssi_hist.append(rssi)
        self._snr_hist.append(snr)
        self._quality_hist.append(quality)
        return FrameMetrics(
            timestamp=time.time(), frame_index=self._frame_idx,
            noise_level=round(noise, 2), contrast=round(contrast, 2),
            temporal_delta=round(temporal, 2), dropout_ratio=round(dropout, 4),
            blocking_score=round(blocking, 3), ssim_score=round(ssim, 4),
            signal_quality=round(quality, 1), rssi_proxy_dbm=rssi,
            snr_db=snr, sync_lost=sync_lost,
            rssi_avg=round(float(np.mean(self._rssi_hist)), 2),
            snr_avg=round(float(np.mean(self._snr_hist)), 2),
            quality_avg=round(float(np.mean(self._quality_hist)), 1),
        )


# ─── Video Capture ────────────────────────────────────────────────────────────
class VideoCapture:
    """
    Wraps cv2.VideoCapture.
    Backend is selected automatically per OS — no CLI override needed:
      Windows → CAP_DSHOW   (handles USB capture cards correctly)
      Linux   → CAP_V4L2    (direct device control)
      Other   → CAP_ANY
    """

    BLACK_THRESHOLD = 8.0

    def __init__(self, device: int, width: int = 640, height: int = 480,
                 warmup: int = 30):
        self.device      = device
        self.width       = width
        self.height      = height
        self._warmup     = warmup
        self._cap        = None
        self._fps_history = deque(maxlen=30)
        self._last_ts    = 0.0
        self.black_count = 0

    def open(self):
        # Auto-select best backend per OS
        if sys.platform == "win32":
            backend = cv2.CAP_DSHOW
            log.info("Backend: DirectShow (Windows)")
        elif sys.platform == "linux":
            backend = cv2.CAP_V4L2
            log.info("Backend: V4L2 (Linux)")
        else:
            backend = cv2.CAP_ANY
            log.info("Backend: auto")

        self._cap = cv2.VideoCapture(self.device, backend)
        if not self._cap.isOpened():
            raise RuntimeError(
                f"Cannot open video device {self.device}.\n"
                f"Run: python diagnose_capture.py --device {self.device}"
            )

        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap.set(cv2.CAP_PROP_FPS, 30)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        actual_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        log.info(f"Device {self.device} opened: {actual_w}×{actual_h}")

        log.info(f"Flushing {self._warmup} warm-up frames…")
        for _ in range(self._warmup):
            self._cap.grab()
        log.info("Warm-up done. Preview should now show video.")

    def read(self) -> Optional[np.ndarray]:
        if not self._cap:
            return None
        if not self._cap.grab():
            return None
        ret, frame = self._cap.retrieve()
        if not ret or frame is None:
            return None
        if frame.mean() < self.BLACK_THRESHOLD:
            self.black_count += 1
            return None
        now = time.time()
        if self._last_ts:
            dt = now - self._last_ts
            if dt > 0:
                self._fps_history.append(1.0 / dt)
        self._last_ts = now
        return frame

    @property
    def fps(self) -> float:
        return float(np.mean(self._fps_history)) if self._fps_history else 0.0

    def release(self):
        if self._cap:
            self._cap.release()


# ─── ESP32 WiFi UDP Relay ─────────────────────────────────────────────────────
class ESP32Relay:
    """
    Sends RSSI/SNR to the ESP32 over UDP and receives VTX status replies.

    Outgoing (laptop → ESP32 port 5005):
      {"rssi": -72.5, "snr": 18.3, "ts": 1234567}

    Incoming (ESP32 → laptop port 5005):
      {"type":"vtx_status","action":"power_up","pwr_mw":800,...}
    """

    RELAY_HZ  = 10
    UDP_PORT  = 5005

    def __init__(self, esp32_ip: str, port: int = 5005):
        self._dest      = (esp32_ip, port)
        self._port      = port
        self._sock      = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("0.0.0.0", port))
        self._sock.setblocking(False)
        self._last_send = 0.0
        self._interval  = 1.0 / self.RELAY_HZ
        self.vtx_status: dict = {}
        self.tx_count   = 0
        self.rx_count   = 0
        log.info(f"ESP32 UDP relay → {esp32_ip}:{port}  (connect to AP first)")

    def send(self, m: "FrameMetrics") -> bool:
        now = time.time()
        if now - self._last_send < self._interval:
            return False
        frame = {"rssi": m.rssi_avg, "snr": m.snr_avg, "ts": int(m.timestamp * 1000)}
        try:
            self._sock.sendto(json.dumps(frame).encode(), self._dest)
            self._last_send = now
            self.tx_count  += 1
            return True
        except OSError as e:
            log.warning(f"ESP32 UDP send error: {e}")
            return False

    def read_status(self) -> Optional[dict]:
        try:
            data, _ = self._sock.recvfrom(512)
            obj = json.loads(data.decode())
            if obj.get("type") == "vtx_status":
                self.vtx_status = obj
                self.rx_count  += 1
                return obj
        except BlockingIOError:
            pass   # no data waiting — normal
        except (json.JSONDecodeError, OSError):
            pass
        return None

    def close(self):
        self._sock.close()


# ─── CSV Logger ───────────────────────────────────────────────────────────────
class MetricsLogger:
    FIELDS = [
        "timestamp", "frame_index", "noise_level", "contrast",
        "temporal_delta", "dropout_ratio", "blocking_score", "ssim_score",
        "signal_quality", "rssi_proxy_dbm", "snr_db", "sync_lost",
        "rssi_avg", "snr_avg", "quality_avg",
    ]

    def __init__(self, path: str):
        self._fh     = open(path, "w", newline="")
        self._writer = csv.DictWriter(self._fh, fieldnames=self.FIELDS)
        self._writer.writeheader()
        log.info(f"Logging → {path}")

    def write(self, m: FrameMetrics):
        row = asdict(m)
        row["timestamp"] = datetime.fromtimestamp(m.timestamp).isoformat()
        self._writer.writerow(row)
        self._fh.flush()

    def close(self):
        self._fh.close()


# ─── Rich Dashboard ───────────────────────────────────────────────────────────
BARS = " ▁▂▃▄▅▆▇█"

def _spark(vals, w=45):
    if not vals:
        return "─" * w
    t  = list(vals)[-w:]
    lo, hi = min(t), max(t)
    r  = (hi - lo) or 1.0
    return "".join(BARS[int((v - lo) / r * (len(BARS) - 1))] for v in t)

def _bar(v, lo, hi, w=22):
    filled = int(max(0, min(w, (v - lo) / max(hi - lo, 1) * w)))
    col    = "green" if filled > 2*w//3 else ("yellow" if filled > w//3 else "red")
    return f"[{col}]{'█'*filled}{'░'*(w-filled)}[/{col}]"

def _col(v, bad, good):
    return "green" if v >= good else ("yellow" if v >= bad else "red")


def render_dashboard(latest: Optional[FrameMetrics], history: dict, fps: float,
                     vtx: Optional[dict] = None, relay_tx: int = 0,
                     relay_rx: int = 0, black_count: int = 0) -> Panel:
    if latest is None:
        return Panel("[dim]Waiting for frames…[/dim]",
                     title="[bold cyan]OpenCV Video Quality Analyzer[/bold cyan]")

    t = Table(box=box.SIMPLE, show_header=False, padding=(0, 1), expand=True)
    t.add_column(style="dim",       width=19)
    t.add_column(min_width=24)
    t.add_column(style="bold white", width=16, justify="right")

    rc   = _col(latest.rssi_proxy_dbm, -85, -65)
    sc   = _col(latest.snr_db,          10,  20)
    qc   = _col(latest.signal_quality,  40,  70)
    sync = "[red]LOST ✗[/red]" if latest.sync_lost else "[green]OK ✓[/green]"

    t.add_row("RSSI Proxy",     _bar(latest.rssi_proxy_dbm, -120, 0),
              f"[{rc}]{latest.rssi_proxy_dbm:.1f} dBm[/{rc}]")
    t.add_row("  5s avg",       "", f"[cyan]{latest.rssi_avg:.1f} dBm[/cyan]")
    t.add_row("SNR",            _bar(latest.snr_db, 0, 40),
              f"[{sc}]{latest.snr_db:.1f} dB[/{sc}]")
    t.add_row("  5s avg",       "", f"[cyan]{latest.snr_avg:.1f} dB[/cyan]")
    t.add_row("Signal Quality", _bar(latest.signal_quality, 0, 100),
              f"[{qc}]{latest.signal_quality:.1f} / 100[/{qc}]")
    t.add_row("Sync",           sync, "")
    t.add_row("")

    nc = _col(100 - latest.noise_level / NOISE_BAD * 100, 30, 70)
    t.add_row("Noise (Lap.var)", _bar(100 - latest.noise_level/NOISE_BAD*100, 0, 100),
              f"[{nc}]{latest.noise_level:.0f}[/{nc}]")
    t.add_row("Contrast",        _bar(latest.contrast, 0, 80), f"{latest.contrast:.1f}")
    t.add_row("Temporal Δ",      _bar(100 - latest.temporal_delta/TEMPORAL_NOISY*100, 0, 100),
              f"{latest.temporal_delta:.1f}")
    t.add_row("Dropout",         _bar(100 - latest.dropout_ratio*100/DROPOUT_MAX, 0, 100),
              f"{latest.dropout_ratio*100:.1f}%")
    t.add_row("SSIM",            _bar(latest.ssim_score*100, 0, 100),
              f"{latest.ssim_score:.3f}")
    t.add_row("Blocking",        "", f"[dim]{latest.blocking_score:.2f}[/dim]")
    t.add_row("")

    t.add_row("RSSI trend",    _spark(history["rssi"],    45), "")
    t.add_row("SNR trend",     _spark(history["snr"],     45), "")
    t.add_row("Quality trend", _spark(history["quality"], 45), "")
    t.add_row("")

    if vtx:
        action  = vtx.get("action", "—")
        pwr_mw  = vtx.get("pwr_mw",  "—")
        freq    = vtx.get("freq",    "—")
        state   = vtx.get("state",   "—")
        confirm = vtx.get("confirm", False)
        ac = {"power_up":"yellow","power_down":"cyan","channel_hop":"magenta",
              "failsafe":"red","hold":"dim"}.get(action, "white")
        t.add_row("[bold]── ESP32 VTX ──[/bold]", "", "")
        t.add_row("Action",    f"[{ac}]{action}[/{ac}]", "")
        t.add_row("TX Power",  "", f"[yellow]{pwr_mw} mW[/yellow]")
        t.add_row("Frequency", "", f"[cyan]{freq} MHz[/cyan]")
        t.add_row("State",     f"[dim]{state}[/dim]",
                  "[green]confirmed[/green]" if confirm else "[dim]unconfirmed[/dim]")
        t.add_row("Relay",     f"[dim]↑{relay_tx} ↓{relay_rx}[/dim]", "")
    else:
        t.add_row("[dim]── ESP32: not connected ──[/dim]", "", "")

    t.add_row("")
    t.add_row("Frame #",      "", f"[dim]{latest.frame_index}[/dim]")
    t.add_row("Capture FPS",  "", f"[dim]{fps:.1f}[/dim]")
    if black_count > 0:
        t.add_row("Black frames", "", f"[yellow]{black_count}[/yellow]")

    ts = datetime.fromtimestamp(latest.timestamp).strftime("%H:%M:%S")
    return Panel(t,
                 title=f"[bold cyan]OpenCV Video Quality Analyzer[/bold cyan]  [dim]{ts}[/dim]",
                 border_style="cyan")


# ─── Preview Window ───────────────────────────────────────────────────────────
def annotate_preview(frame: np.ndarray, m: FrameMetrics,
                     vtx: Optional[dict] = None) -> np.ndarray:
    vis   = frame.copy()
    h, w  = vis.shape[:2]
    bar_h = 110 if vtx else 80
    overlay = vis.copy()
    cv2.rectangle(overlay, (0, 0), (w, bar_h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.6, vis, 0.4, 0, vis)

    q   = m.signal_quality
    col = (0, 220, 0) if q > 70 else ((0, 200, 200) if q > 40 else (0, 0, 220))

    def txt(s, x, y, scale=0.52, thickness=1, color=None):
        cv2.putText(vis, s, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                    scale, color or col, thickness, cv2.LINE_AA)

    txt(f"RSSI: {m.rssi_proxy_dbm:.1f} dBm", 8,    20)
    txt(f"SNR:  {m.snr_db:.1f} dB",          8,    40)
    txt(f"Q:    {m.signal_quality:.0f}/100",  8,    60)
    txt(f"NOISE:{m.noise_level:.0f}",         w//2, 20)
    txt(f"SSIM: {m.ssim_score:.3f}",          w//2, 40)
    txt(f"DROP: {m.dropout_ratio*100:.1f}%",  w//2, 60)

    if vtx:
        action = vtx.get("action", "—")
        pwr    = vtx.get("pwr_mw", "—")
        freq   = vtx.get("freq",   "—")
        state  = vtx.get("state",  "—")
        ac = ((0,220,0) if action in ("hold","power_down") else
              (0,200,200) if action == "power_up" else
              (0,100,255) if action == "channel_hop" else (0,0,220))
        txt(f"VTX: {action}  {pwr}mW  {freq}MHz  [{state}]",
            8, 90, scale=0.48, color=ac)

    if m.sync_lost:
        cv2.putText(vis, "SYNC LOST", (w//2 - 70, h//2),
                    cv2.FONT_HERSHEY_DUPLEX, 1.2, (0, 0, 255), 2, cv2.LINE_AA)
    return vis


# ─── Main Application ─────────────────────────────────────────────────────────
class VideoQualityAnalyzer:

    ANALYSIS_EVERY_N = 3

    def __init__(self, args):
        self.args     = args
        self.capture  = VideoCapture(args.device, args.width, args.height,
                                     warmup=args.warmup)
        self.analyzer = FrameAnalyzer()
        self.logger   = MetricsLogger(args.log) if args.log else None

        self.relay: Optional[ESP32Relay] = None
        if args.telem:
            try:
                self.relay = ESP32Relay(args.telem)
            except OSError as e:
                log.error(f"Cannot open UDP socket for {args.telem}: {e}")
                log.error("Continuing without ESP32 relay.")

        self._latest: Optional[FrameMetrics] = None
        self._history = {
            "rssi":    deque(maxlen=HISTORY),
            "snr":     deque(maxlen=HISTORY),
            "quality": deque(maxlen=HISTORY),
        }
        self._frame_count  = 0
        self._metric_count = 0

    def run(self):
        self.capture.open()
        log.info("Video quality analyzer running. Press 'q' in preview or Ctrl+C to stop.")
        if self.relay:
            log.info("ESP32 relay active — sending RSSI/SNR to ESP32")
        else:
            log.info("No ESP32 relay — run with --telem ESP32_IP to enable")

        if self.args.show_preview:
            self._start_preview_thread()

        if self.args.dashboard:
            self._run_with_dashboard()
        else:
            self._run_console()

    def _tick(self) -> Optional[FrameMetrics]:
        frame = self.capture.read()
        if frame is None:
            return None

        self._frame_count += 1

        if self._frame_count % self.ANALYSIS_EVERY_N != 0:
            if self.args.show_preview and self._latest:
                self._push_preview(frame, self._latest)
            return None

        m = self.analyzer.process(frame)
        self._latest = m
        self._metric_count += 1

        self._history["rssi"].append(m.rssi_proxy_dbm)
        self._history["snr"].append(m.snr_db)
        self._history["quality"].append(m.signal_quality)

        if self.relay:
            self.relay.send(m)
            self.relay.read_status()

        if self.logger:
            self.logger.write(m)

        if self.args.show_preview:
            self._push_preview(frame, m)

        return m

    def _start_preview_thread(self):
        import threading
        self._preview_frame: Optional[np.ndarray] = None
        self._preview_lock  = threading.Lock()
        self._preview_stop  = threading.Event()

        def _preview_loop():
            cv2.namedWindow("VTX Video Feed — Quality Overlay",
                            cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
            cv2.resizeWindow("VTX Video Feed — Quality Overlay", 960, 540)
            while not self._preview_stop.is_set():
                with self._preview_lock:
                    frame = self._preview_frame
                if frame is not None:
                    cv2.imshow("VTX Video Feed — Quality Overlay", frame)
                key = cv2.waitKey(16) & 0xFF
                if key == ord("q"):
                    self._preview_stop.set()
                    break
            cv2.destroyAllWindows()

        t = threading.Thread(target=_preview_loop, daemon=True)
        t.start()
        self._preview_thread = t

    def _push_preview(self, frame: np.ndarray, m: FrameMetrics):
        vtx = self.relay.vtx_status if self.relay else {}
        vis = annotate_preview(frame, m, vtx)
        with self._preview_lock:
            self._preview_frame = vis
        if self._preview_stop.is_set():
            raise KeyboardInterrupt

    def _show_preview(self, frame: np.ndarray, m: FrameMetrics):
        vtx = self.relay.vtx_status if self.relay else {}
        vis = annotate_preview(frame, m, vtx)
        cv2.imshow("VTX Video Feed — Quality Overlay", vis)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            raise KeyboardInterrupt

    def _run_console(self):
        print(f"\n{'TIME':>8}  {'RSSI':>7}  {'SNR':>6}  {'QUALITY':>7}  "
              f"{'NOISE':>7}  {'SSIM':>6}  {'DROP%':>6}  {'SYNC':>5}  "
              f"{'VTX_ACTION':>12}  {'POWER':>7}")
        print("─" * 90)
        try:
            while True:
                m = self._tick()
                if m:
                    ts   = datetime.fromtimestamp(m.timestamp).strftime("%H:%M:%S")
                    sync = "LOST" if m.sync_lost else "OK"
                    vtx  = self.relay.vtx_status if self.relay else {}
                    act  = vtx.get("action", "—")
                    pwr  = f"{vtx.get('pwr_mw','—')}mW" if vtx else "—"
                    print(f"{ts:>8}  {m.rssi_proxy_dbm:>7.1f}  {m.snr_db:>6.1f}  "
                          f"{m.signal_quality:>7.1f}  {m.noise_level:>7.0f}  "
                          f"{m.ssim_score:>6.3f}  {m.dropout_ratio*100:>5.1f}%  "
                          f"{sync:>5}  {act:>12}  {pwr:>7}")
        except KeyboardInterrupt:
            pass
        finally:
            self._shutdown()

    def _run_with_dashboard(self):
        console = Console()
        try:
            with Live(render_dashboard(None, self._history, 0.0),
                      refresh_per_second=4, screen=True, console=console) as live:
                while True:
                    self._tick()
                    vtx      = self.relay.vtx_status if self.relay else None
                    relay_tx = self.relay.tx_count   if self.relay else 0
                    relay_rx = self.relay.rx_count   if self.relay else 0
                    live.update(render_dashboard(
                        self._latest, self._history, self.capture.fps,
                        vtx, relay_tx, relay_rx, self.capture.black_count,
                    ))
        except KeyboardInterrupt:
            pass
        finally:
            self._shutdown()

    def _shutdown(self):
        self.capture.release()
        if self.args.show_preview:
            self._preview_stop.set()
        cv2.destroyAllWindows()
        if self.relay:
            self.relay.close()
        if self.logger:
            self.logger.close()
        log.info(f"Done. {self._frame_count} frames captured, "
                 f"{self._metric_count} analyzed.")


# ─── CLI ──────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    p = argparse.ArgumentParser(
        description="OpenCV video quality analyzer — direct ESP32 telemetry relay",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python analyzer.py --device 0 --show-preview
  python analyzer.py --device 0 --dashboard --show-preview
  python analyzer.py --device 0 --telem 192.168.4.1 --dashboard --show-preview
  python analyzer.py --device 0 --telem 192.168.4.1 --log flight.csv
        """
    )
    p.add_argument("--device",      type=int, default=0,
                   help="Video capture device index (default: 0)")
    p.add_argument("--width",       type=int, default=640,
                   help="Capture width in pixels (default: 640)")
    p.add_argument("--height",      type=int, default=480,
                   help="Capture height in pixels (default: 480)")
    p.add_argument("--telem",       default=None, metavar="IP",
                   help="ESP32 AP IP address (default 192.168.4.1). "
                        "Connect laptop to the ESP32 WiFi AP first.")
    p.add_argument("--warmup",      type=int, default=30, metavar="N",
                   help="Warm-up frames to discard on startup (default: 30). "
                        "Increase to 60-90 if preview is initially black.")
    p.add_argument("--log",         default=None, metavar="FILE",
                   help="CSV log file path (e.g. flight_001.csv)")
    p.add_argument("--dashboard",   action="store_true",
                   help="Show live Rich terminal dashboard")
    p.add_argument("--show-preview",action="store_true",
                   help="Show OpenCV preview window with metric overlay")

    args = p.parse_args()
    VideoQualityAnalyzer(args).run()
