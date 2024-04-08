from ..ltpacket import *
import time
import argparse


def main():
    parser = argparse.ArgumentParser(description='A simple blink example')
    parser.add_argument('--period', default=0.2, help='Blink period', type=float)
    args = parser.parse_args()

    conn = LTZmq()
    led = Led(0, 1)
    while True:
        conn.send(led)
        led.state = not led.state
        time.sleep(args.period)


if __name__ == "__main__":
    main()
