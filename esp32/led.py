# led.py
# ──────────────────────────────────────────────────────────────────────────────
# LED status indicator.
# Blink patterns communicate system state without needing a serial monitor.
#
# Pattern key (blinks per 2-second window):
#   1 slow blink  — normal operation, good link
#   2 fast blinks — marginal link (power adjustment active)
#   3 fast blinks — failsafe (telemetry lost)
#   solid ON      — booting / initialising
#   rapid flash   — channel hopping
# ──────────────────────────────────────────────────────────────────────────────

import machine
import utime
from config import LED_PIN


class StatusLED:
    PATTERN_BOOT      = "boot"
    PATTERN_GOOD      = "good"
    PATTERN_MARGINAL  = "marginal"
    PATTERN_FAILSAFE  = "failsafe"
    PATTERN_HOP       = "hop"

    def __init__(self):
        self._pin    = machine.Pin(LED_PIN, machine.Pin.OUT)
        self._pattern = self.PATTERN_BOOT
        self._tick    = 0
        self._pin.value(1)   # solid on during boot

    def set_pattern(self, pattern: str):
        if pattern != self._pattern:
            self._pattern = pattern
            self._tick    = 0

    def update(self):
        """
        Call once per main loop tick (100 ms).
        Updates LED state according to current pattern.
        """
        t = self._tick

        if self._pattern == self.PATTERN_BOOT:
            self._pin.value(1)

        elif self._pattern == self.PATTERN_GOOD:
            # 1 blink every 2 s (20 ticks): ON for tick 0, OFF for 1-19
            self._pin.value(1 if t == 0 else 0)

        elif self._pattern == self.PATTERN_MARGINAL:
            # 2 quick blinks: ON at 0,2 OFF at 1,3 then OFF rest of 20 ticks
            self._pin.value(1 if t in (0, 2) else 0)

        elif self._pattern == self.PATTERN_FAILSAFE:
            # 3 quick blinks then long off
            self._pin.value(1 if t in (0, 2, 4) else 0)

        elif self._pattern == self.PATTERN_HOP:
            # Rapid 5-flash burst
            self._pin.value(1 if t % 2 == 0 and t < 10 else 0)

        self._tick = (self._tick + 1) % 20

    def on(self):
        self._pin.value(1)

    def off(self):
        self._pin.value(0)
