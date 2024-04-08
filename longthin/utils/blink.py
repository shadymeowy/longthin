import argparse

from ..node import LTNode
from ..rate import Rate
from ..ltpacket import *


def main():
    parser = argparse.ArgumentParser(description='A simple blink example')
    parser.add_argument('--period', default=0.2, help='Blink period', type=float)
    args = parser.parse_args()

    node = LTNode()
    rate = node.rate(args.period)
    packet = Led(0, 0)
    while True:
        packet.state = not packet.state
        node.publish(packet)
        rate.sleep()


if __name__ == "__main__":
    main()
