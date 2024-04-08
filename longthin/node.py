import os
import time

from .ltpacket import *


class LTNode:
    def __init__(self):
        if "LTZMQ_PORT1" in os.environ:
            self.port1 = int(os.environ["LTZMQ_PORT1"])
        else:
            self.port1 = 5555
        if "LTZMQ_PORT2" in os.environ:
            self.port2 = int(os.environ["LTZMQ_PORT2"])
        else:
            self.port2 = 5556
        self.conn = LTZmq(self.port1, self.port2, server=False)
        self.subscribers = {None: list()}
        self.params = Parameters.from_default()

    def publish(self, msg):
        self.conn.send(msg)

    def read(self):
        return self.conn.read()

    def close(self):
        pass

    def subscribe(self, msg_type, callback):
        if msg_type not in self.subscribers:
            self.subscribers[msg_type] = []
        self.subscribers[msg_type].append(callback)
        return callback

    def unsubscribe(self, msg_type, callback):
        if msg_type in self.subscribers:
            self.subscribers[msg_type].remove(callback)
        else:
            raise Exception("No subscriber for message type with callback")

    def spin_once(self):
        while True:
            packet = self.conn.read()
            if packet is None:
                break
            msg_type = type(packet)
            if msg_type in self.subscribers:
                for callback in self.subscribers[msg_type]:
                    callback(packet)
            for callback in self.subscribers[None]:
                callback(packet)
            if isinstance(packet, Setparam):
                self.params[LTParams(packet.param)] = packet.value
            elif isinstance(packet, Setparami):
                self.params[LTParams(packet.param)] = packet.value
            elif isinstance(packet, Setparamu):
                self.params[LTParams(packet.param)] = packet.value

    def spin(self):
        while True:
            self.spin_once()
            time.sleep(1e-4)
