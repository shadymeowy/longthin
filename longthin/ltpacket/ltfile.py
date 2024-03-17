import struct
import time

from .helpers import *
from .ltpacket_def import *


class LTFileReader:
    def __init__(self, path, timestamp=True):
        self.file = open(path, 'rb')
        self.timestamp = timestamp

    def write(self, packet):
        raise NotImplementedError

    def read(self):
        if not self.is_available():
            return None
        if self.timestamp:
            t = self.file.read(8)
            if t == b'':
                print(t)
                return None
            t = struct.unpack('d', t)[0]
        byts = self.file.read(1)
        if byts[0] != LTPACKET_MAGIC:
            return None
        byts += self.file.read(2)
        length = int.from_bytes(byts[1:3], 'little') - 5
        if length > LTPACKET_MAX_LENGTH:
            return None
        byts += self.file.read(length)
        byts += self.file.read(2)
        try:
            packet = decode(byts)
        except ValueError as e:
            raise ValueError('Invalid packet', e)
        if self.timestamp:
            return t, packet
        else:
            return packet

    def is_available(self):
        return self.file.peek(1) != b''


class LTFileWriter:
    def __init__(self, path, timestamp=True):
        self.file = open(path, 'wb')
        self.timestamp = timestamp

    def write(self, packet):
        if self.timestamp:
            t = struct.pack('d', time.time())
            self.file.write(t)
        byts = encode(packet)
        self.file.write(byts)
        self.file.flush()

    def read(self):
        raise NotImplementedError

    def is_available(self):
        return False