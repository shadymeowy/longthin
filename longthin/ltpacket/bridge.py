from .ltserial import LTSerial
from .ltzmq import LTZmq
import time
import argparse
import yaml
from dataclasses import asdict


def main():
    parser = argparse.ArgumentParser(description='Bridge between serial and zmq for ltpackets')
    parser.add_argument('serial', help='Serial port')
    parser.add_argument('--zmq', default=5555, help='ZMQ port')
    parser.add_argument('--zmq2', default=5556, help='ZMQ port2')
    parser.add_argument('--baudrate', default=115200, help='Serial baudrate')
    parser.add_argument('--debug', action='store_true', help='Print debug messages')
    args = parser.parse_args()
    ser = LTSerial(args.serial, baudrate=args.baudrate)
    conn = LTZmq(args.zmq, args.zmq2, server=True)
    while True:
        if ser.is_available():
            packet = ser.read()
            conn.send(packet)
            if args.debug:
                print(str(type(packet)), time.time())
                print(yaml.dump(asdict(packet)), end='')
        packet = conn.read()
        if packet is not None:
            ser.write(packet)
            if args.debug:
                print(str(type(packet)), time.time())
                print(yaml.dump(asdict(packet)), end='')
        time.sleep(0)


if __name__ == "__main__":
    main()
