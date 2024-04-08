from ..ltpacket import *
import time
import argparse


def main():
    parser = argparse.ArgumentParser(description='A packet frequency monitor')
    parser.add_argument('packet_type', default=None, help='Filter packet type')
    parser.add_argument('--refresh', default=1, help='Refresh rate')
    args = parser.parse_args()
    conn = LTZmq()

    typ = LTPACKET_TYPE[args.packet_type.upper()]
    last_time = time.time()
    count = 0
    while True:
        while True:
            packet = conn.read()
            if packet is None:
                time.sleep(1e-4)
                break
            if packet.type != typ:
                break
            count += 1
        t = time.time()
        dt = t - last_time
        if dt > args.refresh:
            print(f"{count/dt} Hz")
            last_time = t
            count = 0


if __name__ == "__main__":
    main()
