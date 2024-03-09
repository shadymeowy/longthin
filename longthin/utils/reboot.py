from ..ltpacket import *
import time
import argparse
import yaml
from dataclasses import asdict


def main():
    parser = argparse.ArgumentParser(description='Reboot the microcontrollers')
    parser.add_argument('--zmq', default=5555, help='ZMQ port')
    parser.add_argument('--zmq2', default=5556, help='ZMQ port2')
    parser.add_argument('--period', default=0.2, help='Blink period', type=float)
    parser.add_argument('--debug', action='store_true', help='Print debug messages')
    args = parser.parse_args()
    conn = LTZmq(args.zmq, args.zmq2, server=False)
    reboot = Reboot(0)
    while conn.read() is None:
        time.sleep(1e-4)
    conn.send(reboot)


if __name__ == "__main__":
    main()
