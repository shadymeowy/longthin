from ..node import LTNode
from ..ltpacket import LTPacketType, type_map
from ..ltpacket.helpers import encode
import time
import argparse


def main():
    parser = argparse.ArgumentParser(description='A packet frequency monitor')
    parser.add_argument('packet_type', default=None, help='Filter packet type', nargs='?')
    parser.add_argument('--refresh', default=1, help='Refresh rate')
    args = parser.parse_args()

    node = LTNode()

    count = 0
    count_bytes = 0
    last_time = time.time()

    def callback(packet):
        nonlocal count, last_time, count_bytes
        count += 1
        count_bytes += len(encode(packet))
        t = time.time()
        dt = t - last_time
        if dt > args.refresh:
            print(f"{count/dt} Hz, {count_bytes/dt/1000} KB/s")
            last_time = t
            count = 0
            count_bytes = 0

    if args.packet_type is not None:
        name = args.packet_type.upper()
        typ = LTPacketType[name].to_type()
        print(f"Monitoring {name} packets")
    else:
        print("Monitoring all packets")
        typ = None
    node.subscribe(typ, callback)
    node.spin()


if __name__ == "__main__":
    main()
