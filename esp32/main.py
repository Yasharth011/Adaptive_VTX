# main.py
# ──────────────────────────────────────────────────────────────────────────────
# Closed-Loop Adaptive VTX System — ESP32-S3 Main Entry Point
#
# Boot sequence:
#   1. Check BOOT button — if held, enter safe mode
#   2. Load last-known power/channel from NVM flash
#   3. Connect to WiFi, open UDP socket
#   4. Initialise SmartAudio, apply saved state
#   5. Query VTX for confirmation (GET_SETTINGS)
#   6. Enter 10 Hz adaptive control loop
#
# Main loop (every 100 ms):
#   a. Poll telemetry UDP socket for incoming RSSI/SNR frame from laptop
#   b. Feed RSSI/SNR into AdaptiveController.update()
#   c. Controller issues SmartAudio commands if thresholds crossed
#   d. Save new state to NVM if it changed
#   e. Send status frame back to laptop every 500 ms
#   f. Update LED pattern
#   g. Feed hardware watchdog
#
# Modules:
#   config.py      — all pin assignments and tunable parameters
#   smartaudio.py  — SmartAudio v2 UART driver (send + parse responses)
#   controller.py  — hysteresis control loop + failsafe state machine
#   telemetry.py   — JSON-over-WiFi-UDP link to laptop
#   nvm.py         — flash persistence for power/channel state
#   led.py         — LED blink patterns for visual status
# ──────────────────────────────────────────────────────────────────────────────

import machine
import utime
import ujson

from config import (
    AP_SSID,
    BOOT_BTN_PIN,
    LOOP_PERIOD_MS,
    STATUS_EVERY_TICKS,
    TELEM_TIMEOUT_MS,
    FAILSAFE_POWER_IDX,
)
from smartaudio  import SmartAudioV2
from controller  import AdaptiveController
from telemetry   import TelemetryLink
from nvm         import load as nvm_load, save as nvm_save, clear as nvm_clear
from led         import StatusLED


# ──────────────────────────────────────────────────────────────────────────────
# Safe-mode boot
# ──────────────────────────────────────────────────────────────────────────────
def _check_safe_mode() -> bool:
    btn     = machine.Pin(BOOT_BTN_PIN, machine.Pin.IN, machine.Pin.PULL_UP)
    held_ms = 0
    while btn.value() == 0 and held_ms < 1500:
        utime.sleep_ms(100)
        held_ms += 100
    if held_ms >= 1000:
        print("[BOOT] Safe mode triggered (BOOT button held)")
        nvm_clear()
        return True
    return False


# ──────────────────────────────────────────────────────────────────────────────
# VTX initialisation
# ──────────────────────────────────────────────────────────────────────────────
def _init_vtx(vtx: SmartAudioV2, saved: dict) -> None:
    power_idx   = saved["power_idx"]
    channel_idx = saved["channel_idx"]
    print(f"[BOOT] Applying power_idx={power_idx}  channel_idx={channel_idx}")
    for attempt in range(3):
        ok_p = vtx.set_power(power_idx)
        utime.sleep_ms(50)
        ok_c = vtx.set_channel(channel_idx)
        utime.sleep_ms(50)
        if ok_p and ok_c:
            print(f"[BOOT] VTX ready: {vtx.power_mw}mW @ {vtx.freq_mhz} MHz")
            return
        print(f"[BOOT] VTX init attempt {attempt+1}/3 — retrying…")
        utime.sleep_ms(200)
    print("[BOOT] WARN: VTX did not confirm init. Proceeding anyway.")


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────
def main():
    led = StatusLED()
    led.on()

    print("=" * 50)
    print("  Closed-Loop Adaptive VTX — ESP32-S3")
    print("=" * 50)

    # ── Safe mode check ───────────────────────────────────────────────────────
    safe_mode = _check_safe_mode()

    # ── Load persisted state ──────────────────────────────────────────────────
    saved = nvm_load()

    # ── Initialise SmartAudio and controller ──────────────────────────────────
    vtx  = SmartAudioV2()
    ctrl = AdaptiveController(vtx)

    utime.sleep_ms(500)   # let VTX finish booting before first command

    if safe_mode:
        print("[SAFE] Entering pit mode — no adaptive control")
        vtx.set_power(0)
        vtx.set_channel(saved["channel_idx"])
    else:
        _init_vtx(vtx, saved)

    utime.sleep_ms(100)
    vtx.get_settings()
    print(f"[BOOT] Confirmed: {vtx.power_mw}mW @ {vtx.freq_mhz} MHz  "
          f"(VTX responded: {vtx.confirmed})")

    # ── Connect WiFi and open UDP socket ─────────────────────────────────────
    telem = TelemetryLink()
    wifi_ok = telem.connect()

    if not wifi_ok:
        # WiFi failed — still run the VTX at failsafe power, no adaptive control
        print("[BOOT] WiFi failed — running at failsafe power, no telemetry")
        vtx.set_power(FAILSAFE_POWER_IDX)
        safe_mode = True
    else:
        print(f"[BOOT] AP ready — connect laptop to WiFi network: {AP_SSID}")

    # ── Hardware watchdog ─────────────────────────────────────────────────────
    wdt = machine.WDT(timeout=8000)

    # ── State tracking for NVM saves ─────────────────────────────────────────
    last_saved_power   = vtx.power_idx
    last_saved_channel = vtx.channel_idx

    # ── Telemetry seed values ─────────────────────────────────────────────────
    last_rssi = -75.0
    last_snr  = 15.0

    tick      = 0
    start_ms  = utime.ticks_ms()

    print("[READY] Entering adaptive control loop")
    led.set_pattern(led.PATTERN_GOOD)

    # ═════════════════════════════════════════════════════════════════════════
    # Main loop — 10 Hz
    # ═════════════════════════════════════════════════════════════════════════
    while True:
        t0 = utime.ticks_ms()

        # ── a. Read telemetry (non-blocking UDP recv) ─────────────────────────
        frame = telem.poll()
        if frame:
            rssi = frame.get('rssi')
            snr  = frame.get('snr')
            if rssi is not None and -120.0 <= float(rssi) <= 0.0:
                last_rssi = float(rssi)
            if snr is not None and -10.0 <= float(snr) <= 60.0:
                last_snr = float(snr)
            ctrl.notify_telem_received()

        # ── b/c. Run control loop ─────────────────────────────────────────────
        action = "safe_mode" if safe_mode else ctrl.update(last_rssi, last_snr)

        # ── d. Persist state if it changed ───────────────────────────────────
        if (vtx.power_idx   != last_saved_power or
                vtx.channel_idx != last_saved_channel):
            if nvm_save(vtx.power_idx, vtx.channel_idx):
                last_saved_power   = vtx.power_idx
                last_saved_channel = vtx.channel_idx

        # ── e. Send status to laptop every STATUS_EVERY_TICKS ────────────────
        if tick % STATUS_EVERY_TICKS == 0:
            # Keep payload small to avoid ENOMEM on UDP send
            telem.send({
                "type":   "vtx_status",
                "action": action,
                "pwr_mw": vtx.power_mw,
                "freq":   vtx.freq_mhz,
                "rssi":   last_rssi,
                "snr":    last_snr,
                "state":  ctrl._state,
                "confirm":vtx.confirmed,
                "ts":     utime.ticks_ms(),
            })

        # ── f. LED pattern ────────────────────────────────────────────────────
        if safe_mode or ctrl._state == AdaptiveController.STATE_FAILSAFE:
            led.set_pattern(led.PATTERN_FAILSAFE)
        elif action == "channel_hop":
            led.set_pattern(led.PATTERN_HOP)
        elif action in ("power_up", "power_down"):
            led.set_pattern(led.PATTERN_MARGINAL)
        else:
            led.set_pattern(led.PATTERN_GOOD)

        led.update()

        # ── g. Feed watchdog ──────────────────────────────────────────────────
        wdt.feed()

        # ── Timing ────────────────────────────────────────────────────────────
        tick += 1
        elapsed = utime.ticks_diff(utime.ticks_ms(), t0)
        sleep   = LOOP_PERIOD_MS - elapsed
        if sleep > 0:
            utime.sleep_ms(sleep)


# ──────────────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    main()
