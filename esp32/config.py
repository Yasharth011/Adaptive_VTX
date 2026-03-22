# config.py
# ──────────────────────────────────────────────────────────────────────────────
# All hardware pin assignments and tunable parameters in one place.
# Edit this file before flashing; do not scatter magic numbers in other modules.
# ──────────────────────────────────────────────────────────────────────────────

# ── Pin assignments ───────────────────────────────────────────────────────────
SA_PIN          = 17    # SmartAudio single-wire (1kΩ series + 1kΩ pull-up to 3V3)
LED_PIN         = 2     # Built-in blue LED (active HIGH on ESP32-S3 DevKit)
BOOT_BTN_PIN    = 0     # BOOT button — hold on power-up to enter safe mode

# ── UART baud rates ───────────────────────────────────────────────────────────
SA_BAUD         = 4800      # SmartAudio v2 spec

# ── WiFi Access Point configuration ──────────────────────────────────────────
# The ESP32 creates its own WiFi network. Connect your laptop to this network.
#
# AP credentials — change these before flashing
AP_SSID         = "VTX-Controller"  # network name visible on laptop
AP_PASSWORD     = "vtx12345"        # min 8 characters (WPA2)
AP_CHANNEL      = 6                 # WiFi channel 1-13 (6 avoids most 5.8GHz VTX interference)

# ESP32 AP address — this is fixed, you don't need to change it
# After connecting your laptop to the AP, the laptop gets 192.168.4.x
# and the ESP32 is always at 192.168.4.1
AP_IP           = "192.168.4.1"

# UDP telemetry port — same port used on both sides
TELEM_UDP_PORT  = 5005

# Laptop's IP on the AP network — always .2 when DHCP assigns the first address
# If your laptop gets a different address, check with: ip addr show wlan0
LAPTOP_IP       = "192.168.4.2"

# ── SmartAudio channel/power defaults ─────────────────────────────────────────
DEFAULT_POWER_IDX   = 2     # 500 mW — sensible middle-ground startup
DEFAULT_CHANNEL_IDX = 32    # Raceband CH1 (5658 MHz)

# AKK Race Ranger power index → milliwatts
POWER_LEVELS = {
    0:   25,
    1:  200,
    2:  500,
    3:  800,
    4: 1000,
    5: 1600,
}
POWER_IDX_MIN = 0
POWER_IDX_MAX = 5

# Failsafe power (index) used when telemetry is lost
FAILSAFE_POWER_IDX  = 3     # 800 mW

# ── 5.8 GHz channel table  (SmartAudio index 0-39 → MHz) ────────────────────
CHANNEL_TABLE = {
    # Band A
    0:5865, 1:5845, 2:5825, 3:5805, 4:5785, 5:5765, 6:5745, 7:5725,
    # Band B
    8:5733, 9:5752, 10:5771, 11:5790, 12:5809, 13:5828, 14:5847, 15:5866,
    # Band E
    16:5705, 17:5685, 18:5665, 19:5645, 20:5885, 21:5905, 22:5925, 23:5945,
    # Band F (Fatshark)
    24:5740, 25:5760, 26:5780, 27:5800, 28:5820, 29:5840, 30:5860, 31:5880,
    # Band R (Raceband)
    32:5658, 33:5695, 34:5732, 35:5769, 36:5806, 37:5843, 38:5880, 39:5917,
}

# Channel pool used by the controller when hopping on interference
CHANNEL_POOL = [32, 33, 34, 35, 36, 37, 38, 39]

# ── Control loop thresholds ───────────────────────────────────────────────────
RSSI_GOOD       = -60.0
RSSI_BAD        = -68.0
SNR_GOOD        =  28.0
SNR_BAD         =  25.0
HYST_UP         = 3
HYST_DOWN       = 6

# ── Telemetry watchdog ────────────────────────────────────────────────────────
TELEM_TIMEOUT_MS    = 3000

# ── Timing ────────────────────────────────────────────────────────────────────
LOOP_PERIOD_MS      = 100
STATUS_EVERY_TICKS  = 5
SA_CMD_DELAY_MS     = 25

# ── NVM persistence ───────────────────────────────────────────────────────────
NVM_FILE            = "vtx_state.json"
