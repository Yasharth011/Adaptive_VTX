# nvm.py
# ──────────────────────────────────────────────────────────────────────────────
# Non-volatile state persistence using the ESP32-S3 flash filesystem.
# Saves last-known power index and channel so the VTX boots into the
# same configuration it was in before a power cycle mid-flight.
# ──────────────────────────────────────────────────────────────────────────────

import ujson
import uos
from config import NVM_FILE, DEFAULT_POWER_IDX, DEFAULT_CHANNEL_IDX


def load() -> dict:
    """
    Load saved VTX state from flash.
    Returns defaults if the file doesn't exist or is corrupted.
    """
    defaults = {
        "power_idx":   DEFAULT_POWER_IDX,
        "channel_idx": DEFAULT_CHANNEL_IDX,
    }
    try:
        with open(NVM_FILE, 'r') as f:
            data = ujson.loads(f.read())
        # Validate keys exist and are ints
        power_idx   = int(data.get("power_idx",   DEFAULT_POWER_IDX))
        channel_idx = int(data.get("channel_idx", DEFAULT_CHANNEL_IDX))
        # Clamp to valid ranges
        power_idx   = max(0, min(5,  power_idx))
        channel_idx = max(0, min(39, channel_idx))
        print(f"[NVM] Loaded: power_idx={power_idx} channel_idx={channel_idx}")
        return {"power_idx": power_idx, "channel_idx": channel_idx}
    except OSError:
        print("[NVM] No saved state found — using defaults")
        return defaults
    except Exception as e:
        print(f"[NVM] Load error ({e}) — using defaults")
        return defaults


def save(power_idx: int, channel_idx: int) -> bool:
    """
    Persist current VTX state to flash.
    Called after every confirmed power or channel change.
    Returns True on success.
    """
    try:
        data = {"power_idx": power_idx, "channel_idx": channel_idx}
        with open(NVM_FILE, 'w') as f:
            f.write(ujson.dumps(data))
        return True
    except Exception as e:
        print(f"[NVM] Save error: {e}")
        return False


def clear() -> None:
    """Erase saved state (called on factory reset / safe-mode boot)."""
    try:
        uos.remove(NVM_FILE)
        print("[NVM] State cleared")
    except OSError:
        pass
