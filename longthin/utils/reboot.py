import time
import argparse

from ..node import LTNode
from ..ltpacket import *

def main():
    parser = argparse.ArgumentParser(description='Reboot the microcontrollers')
    args = parser.parse_args()

    node = LTNode()
    reboot = Reboot(0)
    while node.read() is None:
        time.sleep(1e-2)
    node.publish(reboot)


if __name__ == "__main__":
    main()
