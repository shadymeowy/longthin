from ..ltpacket import *
import time
import argparse
import yaml
from dataclasses import asdict


def main():
    parser = argparse.ArgumentParser(description='Bridge between serial and zmq for ltpackets')
    parser.add_argument('serial', help='Serial port', default='', nargs='?')
    parser.add_argument('--baudrate', default=115200, help='Serial baudrate')
    parser.add_argument('--debug', action='store_true', help='Print debug messages')
    args = parser.parse_args()
    
    if args.serial:
        ser = LTSerial(args.serial, baudrate=args.baudrate)
    else:
        ser = None
    conn = LTZmq(server=True)
    while True:
        while ser and ser.is_available():
            packet = ser.read()
            conn.send(packet)
            if args.debug:
                print(packet.type, time.time())
                print(yaml.dump(asdict(packet)), end='')
        while True:
            packet = conn.read()
            if packet is None:
                break
            if ser:
                ser.write(packet)
            conn.send(packet)
            if args.debug:
                print(packet.type, time.time())
                print(yaml.dump(asdict(packet)), end='')
        time.sleep(1e-4)


if __name__ == "__main__":
    main()
