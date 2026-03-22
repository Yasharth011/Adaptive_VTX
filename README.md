# Closed-Loop Adaptive VTX System

An intelligent wireless video transmission system for FPV drones that automatically adjusts transmitter power and frequency channel in real time based on video quality feedback from the ground station.

## Overview

Traditional FPV VTX systems are configured manually and remain fixed during flight. This system closes the loop — the ground station continuously analyses the incoming video feed using OpenCV, derives link quality metrics (RSSI proxy, SNR), and sends them back to the drone over WiFi. The ESP32-S3 on the drone receives these metrics and uses a hysteresis-based control loop to dynamically adjust the AKK Race Ranger VTX via the SmartAudio v2 protocol.

```
RunCam → AKK VTX → 5.8 GHz RF → Skydroid RX → USB capture card
                                                        ↓
                                              analyzer.py (OpenCV)
                                               7 video quality metrics
                                                        ↓
                                              WiFi UDP (192.168.4.1:5005)
                                                        ↓
                                              ESP32-S3 (MicroPython)
                                               Hysteresis control loop
                                                        ↓
                                              SmartAudio v2 → AKK VTX
                                               Power / channel adjusted
```

---

## Hardware

| Component | Role |
|---|---|
| ESP32-S3 | Adaptive controller, WiFi Access Point |
| AKK Race Ranger VTX (1.6W) | 5.8 GHz video transmitter |
| RunCam analog camera | Video source |
| Skydroid receiver | 5.8 GHz video receiver |
| USB video capture card | AV-OUT → laptop USB |
| Laptop | Ground station, OpenCV analysis |
| LiPo battery | Powers VTX and camera |

---

## Wiring

### AKK VTX → ESP32-S3

```
VTX SA pad ──[1kΩ]──┬── ESP32 GPIO17
                     │
                   [1kΩ]
                     │
                   3.3V (ESP32)

VTX GND ─────────── ESP32 GND   (mandatory — common ground)
VTX 5V  ─────────── LiPo 5V rail (not from ESP32)
```

### RunCam → AKK VTX

```
Camera Video OUT ── VTX CAM IN
Camera 5V        ── 5V rail
Camera GND       ── GND
```

The RunCam's SmartAudio wire connects to the VTX JST connector as supplied. The ESP32 connects to the SA pad on the VTX PCB. Both land on the same bus.

### Ground Station

```
Skydroid AV-OUT (RCA yellow) → USB capture card → Laptop USB
```

---

## Software Architecture

### ESP32 Firmware (MicroPython)

| File | Purpose |
|---|---|
| `config.py` | All pin assignments, WiFi credentials, control thresholds |
| `smartaudio.py` | SmartAudio v2 TX-only driver (CRC-8/DVB-S2, 4800 baud) |
| `controller.py` | Hysteresis control loop + failsafe state machine |
| `telemetry.py` | WiFi Access Point + UDP socket |
| `nvm.py` | Flash persistence — survives power cycles |
| `led.py` | LED blink patterns for visual status |
| `main.py` | Boot sequence and 10 Hz main loop |

### Ground Station (Python 3.10+)

| File | Purpose |
|---|---|
| `analyzer.py` | Main application — OpenCV analysis + ESP32 relay + dashboard |
| `diagnose_capture.py` | Capture card diagnostic tool |

---

## Control Loop

The ESP32 runs at 10 Hz. Each tick it reads the latest RSSI/SNR from the UDP socket and classifies the link into one of three zones:

| Zone | Condition | Action |
|---|---|---|
| Good | RSSI > −65 dBm AND SNR > 20 dB | Reduce power after 6 consecutive good readings |
| Marginal | Between good and bad | Hold, decay counters |
| Bad | RSSI < −85 dBm OR SNR < 10 dB | Increase power after 3 consecutive bad readings |

If power is already at maximum (1600 mW) and the link is still bad, the controller hops to the next Raceband channel instead.

**Power levels (AKK Race Ranger):**
`25 mW → 200 mW → 500 mW → 800 mW → 1000 mW → 1600 mW`

**Channel pool (Raceband):**
`5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917 MHz`

**Failsafe:** If no telemetry is received for 3 seconds, the ESP32 locks power at 800 mW and stops channel hopping until the link recovers.

---

## Video Quality Metrics (OpenCV)

`analyzer.py` processes every 3rd captured frame (~10 Hz at 30 fps) and computes 7 features:

| Metric | Method | What it detects |
|---|---|---|
| Noise level | Laplacian variance | Salt-and-pepper RF noise |
| Contrast | Luma std deviation | Washed-out / crushed image |
| Temporal delta | Mean abs frame diff | Random noise flicker vs real motion |
| Dropout ratio | Clipped pixel fraction | Black / white dropout frames |
| Blocking score | DCT boundary energy | USB recompression artifacts from noise |
| SSIM | Structural similarity vs clean ref | Overall structural degradation |
| Sync loss | Histogram collapse | Complete signal loss / snow |

These are weighted and combined into a quality score (0–100), then mapped linearly to an RSSI proxy (−95 to −55 dBm) and SNR (0–40 dB).

**Feature weights:**
```
Noise    30%   Temporal  20%   Dropout  20%
Contrast 15%   SSIM      10%   Blocking  5%
```

---

## Installation

### ESP32 Firmware

**1. Flash MicroPython** (use the non-SPIRAM build):
```bash
pip install esptool mpremote

# Erase and flash
esptool.py --chip esp32s3 --port /dev/ttyUSB0 erase_flash
esptool.py --chip esp32s3 --port /dev/ttyUSB0 --baud 460800 \
  write_flash -z 0x0 ESP32_GENERIC_S3-vX.XX.X.bin
```

**2. Edit `config.py`** before copying:
```python
AP_SSID      = "VTX-Controller"   # WiFi network name
AP_PASSWORD  = "vtx12345"         # min 8 characters
```

**3. Copy all firmware files:**
```bash
cd esp32_firmware
for file in *.py; do
  mpremote connect /dev/ttyUSB0 fs cp "$file" :
done
```

**4. Verify:**
```bash
mpremote connect /dev/ttyUSB0 fs ls
```

### Ground Station

```bash
pip install opencv-python numpy scipy rich
```

---

## Running the System

### Step 1 — Power the ESP32
Plug into USB. LED blinks slowly. Serial output shows:
```
[WIFI] Access Point started
[WIFI] SSID:     VTX-Controller
[WIFI] ESP32 IP: 192.168.4.1
[READY] Entering adaptive control loop
```

### Step 2 — Power the VTX and camera
Connect LiPo. VTX boots and applies saved power/channel settings.

### Step 3 — Connect laptop to ESP32 WiFi
Connect to `VTX-Controller` in WiFi settings. Laptop gets IP `192.168.4.2`.

### Step 4 — Connect Skydroid and capture card
Plug RCA AV-OUT into capture card, plug capture card into laptop.

### Step 5 — Find capture device index
```bash
python diagnose_capture.py
```

### Step 6 — Run
```bash
python analyzer.py \
  --device 0 \
  --telem 192.168.4.1 \
  --dashboard \
  --show-preview
```

---

## Command Reference

### analyzer.py

```
--device N         Video capture device index (default: 0)
--width N          Capture width in pixels (default: 640)
--height N         Capture height in pixels (default: 480)
--telem IP         ESP32 AP IP address (default: 192.168.4.1)
--warmup N         Warm-up frames to flush on startup (default: 30)
--log FILE         CSV log file path
--dashboard        Show live Rich terminal dashboard
--show-preview     Show OpenCV preview window with HUD overlay
```

### ESP32 safe mode
Hold the BOOT button for 1 second during power-up. This sets the VTX to 25 mW, disables the control loop, and clears NVM state.

---

## Testing Without Flying

**Cover the camera lens** — dropout and noise metrics spike, triggering `power_up` within ~300 ms.

**Inject fake RSSI from laptop:**
```bash
python3 -c "
import socket, json, time
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
for i in range(20):
    s.sendto(json.dumps({'rssi': -95.0, 'snr': 5.0, 'ts': i}).encode(),
             ('192.168.4.1', 5005))
    time.sleep(0.1)
"
```

**Physically block the RF path** — place a metal sheet between VTX and Skydroid antenna.

---

## LED Status Patterns

| Pattern | Meaning |
|---|---|
| Single slow blink (2 s) | Normal — good link |
| 2 quick blinks | Marginal — power adjusting |
| 3 quick blinks | Failsafe — telemetry lost |
| Rapid burst | Channel hopping |
| Solid ON | Booting |

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `OSError: ESP_FAIL` on boot | UART init failure | Check `smartaudio.py` has minimal UART init: `machine.UART(1, baudrate=SA_BAUD, tx=SA_PIN)` |
| `VTX responded: False` | SmartAudio wiring | Check 1kΩ series resistor and 1kΩ pull-up to 3.3V on GPIO17 |
| Preview window black | Qt/display issue | Run `diagnose_capture.py --device 0`; try `--warmup 60` |
| No WiFi AP visible | ESP32 not booted | Check USB power; watch serial output |
| UDP not reaching ESP32 | Wrong network | Confirm laptop is connected to `VTX-Controller`, not home WiFi |
| PSRAM errors on boot | Module has no PSRAM | Harmless — ignore, system continues normally |

---

## Project Structure

```
vtx_system/
├── esp32_firmware/
│   ├── config.py          ← edit WiFi credentials here
│   ├── smartaudio.py      ← SmartAudio v2 TX driver
│   ├── controller.py      ← adaptive control loop
│   ├── telemetry.py       ← WiFi AP + UDP
│   ├── nvm.py             ← flash state persistence
│   ├── led.py             ← LED status patterns
│   └── main.py            ← entry point
└── ground_station/
    ├── analyzer.py        ← main ground station application
    └── diagnose_capture.py← capture card diagnostic
```

---

## Dependencies

### ESP32 (MicroPython built-ins, no pip required)
`machine`, `network`, `socket`, `ujson`, `utime`, `uos`

### Laptop
```
opencv-python
numpy
scipy
rich
```

---

## License

MIT License. See LICENSE file for details.
