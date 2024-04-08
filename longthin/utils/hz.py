from ..node import LTNode
from ..ltpacket import LTPacketType, type_map
import time
import argparse


def main():
    parser = argparse.ArgumentParser(description='A packet frequency monitor')
    parser.add_argument('packet_type', default=None, help='Filter packet type')
    parser.add_argument('--refresh', default=1, help='Refresh rate')
    args = parser.parse_args()

    node = LTNode()

    count = 0
    last_time = time.time()

    def callback(packet):
        nonlocal count, last_time
        count += 1
        t = time.time()
        dt = t - last_time
        if dt > args.refresh:
            print(f"{count/dt} Hz")
            last_time = t
            count = 0

    name = args.packet_type.upper()
    typ = LTPacketType[name]
    typ = type_map[typ]
    print(f"Monitoring {name} packets")
    node.subscribe(typ, callback)
    node.spin()


if __name__ == "__main__":
    main()
