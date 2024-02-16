from ..ltpacket import *
import time
import argparse
import yaml
from dataclasses import asdict


def main():
    parser = argparse.ArgumentParser(description='A packet echo client')
    parser.add_argument('--zmq', default=5555, help='ZMQ port')
    parser.add_argument('--zmq2', default=5556, help='ZMQ port2')
    parser.add_argument('--filter', default=None, help='Filter packets', nargs='+')
    args = parser.parse_args()
    conn = LTZmq(args.zmq, args.zmq2, server=False)
    if args.filter is not None:
        typs = []
        for name in args.filter:
            typ = LTPACKET_TYPE[name.upper()]
            typs.append(typ)
    while True:
        packet = conn.read()
        if packet is not None:
            if args.filter is not None and packet.type not in typs:
                continue
            print(packet.type)
            print(yaml.dump(asdict(packet)), end='')
        time.sleep(1e-4)


if __name__ == "__main__":
    main()
