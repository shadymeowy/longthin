from .ltzmq import LTZmq
from .ltpacket_def import *
import time
import argparse
import yaml
from dataclasses import asdict


def main():
    parser = argparse.ArgumentParser(description='A simple blink example')
    parser.add_argument('--zmq', default=5555, help='ZMQ port')
    parser.add_argument('--zmq2', default=5556, help='ZMQ port2')
    parser.add_argument('--period', default=0.2, help='Blink period', type=float)
    parser.add_argument('--debug', action='store_true', help='Print debug messages')
    args = parser.parse_args()
    conn = LTZmq(args.zmq, args.zmq2, server=False)
    led = Led(0, 0)
    while True:
        conn.send(led)
        if args.debug:
            print(led.type, time.time())
            print(yaml.dump(asdict(led)), end='')
        led.state = not led.state
        time.sleep(args.period)


if __name__ == "__main__":
    main()
