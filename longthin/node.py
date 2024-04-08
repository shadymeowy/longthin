import os
import time

from .ltpacket import LTZmq


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

    def publish(self, msg):
        self.conn.send(msg)

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
            print("No subscriber for message type with callback")

    def spin_once(self):
        while True:
            msg = self.conn.read()
            if msg is None:
                break
            msg_type = type(msg)
            if msg_type in self.subscribers:
                for callback in self.subscribers[msg_type]:
                    callback(msg)
            for callback in self.subscribers[None]:
                callback(msg)

    def spin(self):
        while True:
            self.spin_once()
            time.sleep(1e-4)
