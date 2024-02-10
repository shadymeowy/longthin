from .helpers import *
from .ltpacket_def import *


class LTSerial:
    def __init__(self, port, baudrate):
        import serial
        self.ser = serial.Serial(port, baudrate)

    def write(self, packet):
        byts = encode(packet)
        self.ser.write(byts)

    def read(self):
        while True:
            byts = self.ser.read(1)
            if byts[0] != LTPACKET_MAGIC:
                continue
            byts += self.ser.read(2)
            length = int.from_bytes(byts[1:3], 'little') - 5
            if length > LTPACKET_MAX_LENGTH:
                continue
            byts += self.ser.read(length)
            byts += self.ser.read(2)
            try:
                packet = decode(byts)
            except ValueError as e:
                print('Invalid packet', e)
                continue
            if packet is not None:
                return packet

    def is_available(self):
        return self.ser.in_waiting > 0
