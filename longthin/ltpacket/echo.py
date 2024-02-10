from .ltzmq import LTZmq
from .ltpacket_def import *
import time
import argparse
import yaml
from dataclasses import asdict


def main():
    parser = argparse.ArgumentParser(description='A packet echo client')
    parser.add_argument('--zmq', default=5555, help='ZMQ port')
    parser.add_argument('--zmq2', default=5556, help='ZMQ port2')
    args = parser.parse_args()
    conn = LTZmq(args.zmq, args.zmq2, server=False)
    while True:
        packet = conn.read()
        if packet is not None:
            conn.send(packet)
            print(str(type(packet)), time.time())
            print(yaml.dump(asdict(packet)), end='')
        time.sleep(0)


if __name__ == "__main__":
    main()
