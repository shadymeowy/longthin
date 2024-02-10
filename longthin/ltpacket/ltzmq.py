from .helpers import *
from .ltpacket_def import *


class LTZmq:
    def __init__(self, port, port2, server=False, context=None):
        import zmq
        self.context = zmq.Context()
        self.socket_pub = self.context.socket(zmq.PUB)
        if server:
            self.socket_pub.bind(f"tcp://*:{port}")
        else:
            self.socket_pub.connect(f"tcp://localhost:{port2}")
        self.socket_sub = self.context.socket(zmq.SUB)
        if server:
            self.socket_sub.bind(f"tcp://*:{port2}")
        else:
            self.socket_sub.connect(f"tcp://localhost:{port}")
        self.socket_sub.setsockopt_string(zmq.SUBSCRIBE, '')

    def send(self, packet):
        byts = encode(packet)
        self.socket_pub.send(byts)

    def read(self):
        if self.socket_sub.poll(0) == 0:
            return None
        byts = self.socket_sub.recv()
        packet = decode(byts)
        return packet
