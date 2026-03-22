# controller.py
# ──────────────────────────────────────────────────────────────────────────────
# Closed-loop adaptive controller.
# Reads RSSI + SNR from the telemetry link, decides whether to adjust
# TX power or hop channels, and issues commands to the SmartAudio driver.
# ──────────────────────────────────────────────────────────────────────────────

import utime
from config import (
    RSSI_GOOD, RSSI_BAD, SNR_GOOD, SNR_BAD,
    HYST_UP, HYST_DOWN,
    POWER_IDX_MIN, POWER_IDX_MAX,
    CHANNEL_POOL,
    TELEM_TIMEOUT_MS,
    FAILSAFE_POWER_IDX,
)


class AdaptiveController:
    """
    Hysteresis-based closed-loop VTX controller.

    State machine:
      NORMAL   — actively adjusting power / hopping channels
      FAILSAFE — telemetry lost; hold at FAILSAFE_POWER_IDX, no hops
    """

    STATE_NORMAL   = "normal"
    STATE_FAILSAFE = "failsafe"

    def __init__(self, vtx):
        self._vtx          = vtx
        self._state        = self.STATE_NORMAL

        # Hysteresis counters
        self._bad_count    = 0
        self._good_count   = 0

        # Channel pool pointer
        self._ch_idx       = 0   # index into CHANNEL_POOL list

        # Telemetry watchdog
        self._last_telem_ms = utime.ticks_ms()
        self._telem_lost    = False

        # History for logging
        self.last_action   = "init"
        self.action_count  = {
            "power_up":     0,
            "power_down":   0,
            "channel_hop":  0,
            "failsafe_in":  0,
            "failsafe_out": 0,
        }

    # ── Telemetry watchdog ────────────────────────────────────────────────────
    def notify_telem_received(self):
        """Call this every time a valid telemetry frame arrives."""
        self._last_telem_ms = utime.ticks_ms()
        if self._telem_lost:
            self._exit_failsafe()

    def _check_watchdog(self):
        elapsed = utime.ticks_diff(utime.ticks_ms(), self._last_telem_ms)
        if elapsed > TELEM_TIMEOUT_MS and not self._telem_lost:
            self._enter_failsafe()

    def _enter_failsafe(self):
        self._telem_lost = True
        self._state      = self.STATE_FAILSAFE
        self._bad_count  = 0
        self._good_count = 0
        # Set a fixed mid-range power so we stay reachable
        self._vtx.set_power(FAILSAFE_POWER_IDX)
        self.last_action = "failsafe_in"
        self.action_count["failsafe_in"] += 1
        print(f"[CTRL] FAILSAFE ENTERED — holding at {self._vtx.power_mw}mW")

    def _exit_failsafe(self):
        self._telem_lost = False
        self._state      = self.STATE_NORMAL
        self.last_action = "failsafe_out"
        self.action_count["failsafe_out"] += 1
        print("[CTRL] Telemetry recovered — resuming adaptive control")

    # ── Channel hopping ───────────────────────────────────────────────────────
    def _hop_channel(self):
        self._ch_idx = (self._ch_idx + 1) % len(CHANNEL_POOL)
        new_ch = CHANNEL_POOL[self._ch_idx]
        self._vtx.set_channel(new_ch)
        print(f"[CTRL] Channel hop → idx={new_ch} ({self._vtx.freq_mhz} MHz)")

    # ── Main update ───────────────────────────────────────────────────────────
    def update(self, rssi: float, snr: float) -> str:
        """
        Called once per control loop tick with latest RSSI and SNR.
        Returns action string for telemetry reporting.
        """
        self._check_watchdog()

        if self._state == self.STATE_FAILSAFE:
            return "failsafe"

        action = "hold"

        # ── BAD zone ──────────────────────────────────────────────
        if rssi < RSSI_BAD or snr < SNR_BAD:
            self._bad_count  += 1
            self._good_count  = 0

            if self._bad_count >= HYST_UP:
                self._bad_count = 0

                if self._vtx.power_idx < POWER_IDX_MAX:
                    self._vtx.set_power(self._vtx.power_idx + 1)
                    action = "power_up"
                    self.action_count["power_up"] += 1
                    print(f"[CTRL] Power UP → {self._vtx.power_mw}mW "
                          f"(RSSI={rssi:.1f} SNR={snr:.1f})")
                else:
                    # Power is already maxed — interference, not range
                    self._hop_channel()
                    action = "channel_hop"
                    self.action_count["channel_hop"] += 1

        # ── GOOD zone ─────────────────────────────────────────────
        elif rssi > RSSI_GOOD and snr > SNR_GOOD:
            self._good_count += 1
            self._bad_count   = 0

            if self._good_count >= HYST_DOWN:
                self._good_count = 0

                if self._vtx.power_idx > POWER_IDX_MIN:
                    self._vtx.set_power(self._vtx.power_idx - 1)
                    action = "power_down"
                    self.action_count["power_down"] += 1
                    print(f"[CTRL] Power DOWN → {self._vtx.power_mw}mW "
                          f"(RSSI={rssi:.1f} SNR={snr:.1f})")

        # ── MARGINAL zone — decay counters gently ─────────────────
        else:
            self._bad_count  = max(0, self._bad_count  - 1)
            self._good_count = max(0, self._good_count - 1)

        self.last_action = action
        return action

    # ── Diagnostics ───────────────────────────────────────────────────────────
    def status_dict(self) -> dict:
        return {
            "state":      self._state,
            "bad_count":  self._bad_count,
            "good_count": self._good_count,
            "telem_lost": self._telem_lost,
            "actions":    self.action_count,
        }
