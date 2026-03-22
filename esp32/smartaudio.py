import machine, utime
from config import SA_PIN, SA_BAUD, SA_CMD_DELAY_MS, POWER_LEVELS, CHANNEL_TABLE, POWER_IDX_MIN, POWER_IDX_MAX, DEFAULT_POWER_IDX, DEFAULT_CHANNEL_IDX

_S0=0xAA;_S1=0x55

def _crc8(d):
    c=0
    for b in d:
        c^=b
        for _ in range(8):
            c=((c<<1)^0xD5)&0xFF if(c&0x80)else(c<<1)&0xFF
    return c

class SmartAudioV2:
    def __init__(self):
        self._u=machine.UART(1,baudrate=SA_BAUD,tx=SA_PIN)
        self.power_idx=DEFAULT_POWER_IDX
        self.channel_idx=DEFAULT_CHANNEL_IDX
        self.confirmed=True
        self.cmd_count=0
        self.rsp_count=0
        self.crc_errors=0

    def _build(self,cmd,payload=b''):
        h=bytes([_S0,_S1,cmd,len(payload)])
        b=h+payload
        return b+bytes([_crc8(b[2:])])

    def _send(self,cmd,payload=b''):
        self._u.write(self._build(cmd,payload))
        self.cmd_count+=1
        utime.sleep_ms(SA_CMD_DELAY_MS)
        return True

    def get_settings(self):
        return self._send(0x03)

    def set_power(self,idx):
        idx=max(POWER_IDX_MIN,min(POWER_IDX_MAX,idx))
        self.power_idx=idx
        self._send(0x05,bytes([idx]))
        print('[SA] Power',self.power_mw,'mW')
        return True

    def set_channel(self,idx):
        idx=idx%40
        self.channel_idx=idx
        self._send(0x07,bytes([idx]))
        print('[SA] Chan',self.freq_mhz,'MHz')
        return True

    def set_pit_mode(self,e):
        return self._send(0x09,bytes([0x01 if e else 0x00]))

    @property
    def power_mw(self):
        return POWER_LEVELS.get(self.power_idx,0)

    @property
    def freq_mhz(self):
        return CHANNEL_TABLE.get(self.channel_idx,0)

    def status_dict(self):
        return {
            'power_idx':self.power_idx,
            'power_mw':self.power_mw,
            'channel_idx':self.channel_idx,
            'freq_mhz':self.freq_mhz,
            'confirmed':self.confirmed,
            'cmd_count':self.cmd_count
        }
