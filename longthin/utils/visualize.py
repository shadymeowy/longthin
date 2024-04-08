import numpy as np
import argparse

from ..graphics import LTRenderer
from ..config import load_config
from ..ltpacket import *


def main():
    parser = argparse.ArgumentParser(description='A visualization of the robot')
    args = parser.parse_args()

    conn = LTZmq()
    config = load_config('default.yaml')
    renderer = LTRenderer(config)

    x, y, yaw = 0, 0, 0
    while True:
        while True:
            packet = conn.read()
            if packet is None:
                break
            if isinstance(packet, EvPose):
                x = packet.x
                y = packet.y
                yaw = packet.yaw

        renderer.vehicle_pose.pos = np.array([x, y, 0.])
        renderer.vehicle_pose.att = np.array([0., 0., yaw])

        if renderer.draw():
            break


if __name__ == "__main__":
    main()
