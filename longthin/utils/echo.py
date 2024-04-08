from ..ltpacket import *
import time
import argparse
import yaml
import json
from dataclasses import asdict


def main():
    parser = argparse.ArgumentParser(description='A packet echo client')
    parser.add_argument('--filter', '-f', default=None, help='Filter packets', nargs='+')
    parser.add_argument('--json', '-j', action='store_true', help='Print as json')
    parser.add_argument('--timestamp', '-t', action='store_true', help='Print timestamp')
    args = parser.parse_args()
    
    conn = LTZmq()
    if args.filter is not None:
        typs = []
        for name in args.filter:
            typ = LTPacketType[name.upper()]
            typs.append(typ)
    while True:
        packet = conn.read()
        if packet is not None:
            if args.filter is not None and packet.type not in typs:
                continue
            if args.json:
                dct = {}
                if args.timestamp:
                    dct['timestamp'] = time.time()
                dct['type'] = str(packet.type)
                dct.update(asdict(packet))
                print(json.dumps(dct))
            else:
                print(packet.type)
                if args.timestamp:
                    print(time.time())
                print(yaml.dump(asdict(packet)), end='')
        else:
            time.sleep(1e-4)


if __name__ == "__main__":
    main()
