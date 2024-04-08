import time
import argparse
import yaml
import json
from dataclasses import asdict

from ..node import LTNode
from ..ltpacket import *


def main():
    parser = argparse.ArgumentParser(description='A packet echo client')
    parser.add_argument('--filter', '-f', default=None, help='Filter packets', nargs='+')
    parser.add_argument('--json', '-j', action='store_true', help='Print as json')
    parser.add_argument('--timestamp', '-t', action='store_true', help='Print timestamp')
    args = parser.parse_args()

    node = LTNode()

    def callback(packet):
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

    if args.filter is not None:
        print('Filtering packets', args.filter)
        for name in args.filter:
            typ = LTPacketType[name.upper()]
            node.subscribe(typ.to_type(), callback)
    else:
        print('No filter specified')
        node.subscribe(None, callback)
    node.spin()


if __name__ == "__main__":
    main()
