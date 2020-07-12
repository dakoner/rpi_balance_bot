import pigpio
from read_encoder import decoder

_pi = pigpio.pi()
if not _pi.connected:
    raise IOError("Can't connect to pigpio")

DECODER_LEFT_PINS = (20,21)
DECODER_RIGHT_PINS = (19,16)
class EncoderCallback:
    def __init__(self, name):
        self.name = name
        self.pos = 0
        
    def callback(self, delta):
        self.pos += delta

class Encoders:
    def __init__(self):
        self.ec_left = EncoderCallback("ec_left")
        self.dec_left = decoder(_pi, *DECODER_LEFT_PINS, self.ec_left.callback)
        self.ec_right = EncoderCallback("ec_right")
        self.dec_right = decoder(_pi, *DECODER_RIGHT_PINS, self.ec_right.callback)
