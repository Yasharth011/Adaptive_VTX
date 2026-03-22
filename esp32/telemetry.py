import network, socket, ujson, utime, gc
from config import AP_SSID, AP_PASSWORD, AP_CHANNEL, AP_IP, LAPTOP_IP, TELEM_UDP_PORT

_MAX_PACKET = 512

class TelemetryLink:
    def __init__(self):
        self._ap   = network.WLAN(network.AP_IF)
        self._sock = None
        self._dest = (LAPTOP_IP, TELEM_UDP_PORT)
        self.rx_count=0; self.tx_count=0; self.parse_errors=0; self.connected=False

    def connect(self) -> bool:
        self._ap.active(True)
        self._ap.config(
            ssid=AP_SSID, password=AP_PASSWORD,
            channel=AP_CHANNEL, authmode=network.AUTH_WPA_WPA2_PSK,
        )
        for _ in range(20):
            if self._ap.active(): break
            utime.sleep_ms(100)
        cfg = self._ap.ifconfig()
        print(f"[WIFI] Access Point started")
        print(f"[WIFI] SSID:     {AP_SSID}")
        print(f"[WIFI] Password: {AP_PASSWORD}")
        print(f"[WIFI] ESP32 IP: {cfg[0]}")
        print(f"[WIFI] Connect your laptop to '{AP_SSID}'")
        print(f"[WIFI] Then run: python analyzer.py --telem {cfg[0]}")
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("0.0.0.0", TELEM_UDP_PORT))
        self._sock.setblocking(False)
        self.connected = True
        return True

    def is_connected(self) -> bool:
        try: return len(self._ap.status('stations')) > 0
        except: return self._ap.active()

    def client_count(self) -> int:
        try: return len(self._ap.status('stations'))
        except: return 0

    def poll(self):
        if not self._sock: return None
        try:
            data, addr = self._sock.recvfrom(_MAX_PACKET)
            self._dest = (addr[0], TELEM_UDP_PORT)
            obj = ujson.loads(data)
            self.rx_count += 1
            return obj
        except OSError: return None
        except: self.parse_errors += 1; return None

    def send(self, obj: dict) -> bool:
        if not self._sock: return False
        # Keep status small to avoid ENOMEM — only send essential fields
        try:
            gc.collect()
            payload = ujson.dumps(obj).encode()
            self._sock.sendto(payload, self._dest)
            self.tx_count += 1
            return True
        except OSError as e:
            if e.args[0] == 12:  # ENOMEM
                gc.collect()     # free memory and skip this frame
            else:
                print(f"[TELEM] UDP send error: {e}")
            return False

    def stats(self) -> dict:
        return {"rx":self.rx_count,"tx":self.tx_count,"errors":self.parse_errors,"clients":self.client_count(),"ap_ip":AP_IP}
