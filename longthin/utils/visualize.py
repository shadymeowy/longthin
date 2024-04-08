import numpy as np
import argparse

from ..node import LTNode
from ..graphics import LTRenderer
from ..ltpacket import *


def main():
    parser = argparse.ArgumentParser(description='A visualization of the robot')
    args = parser.parse_args()

    node = LTNode()
    renderer = LTRenderer(node.config)

    x, y, yaw = 0, 0, 0

    def cb_evpose(packet):
        nonlocal x, y, yaw
        x = packet.x
        y = packet.y
        yaw = packet.yaw
        renderer.vehicle_pose.pos = np.array([x, y, 0.])
        renderer.vehicle_pose.att = np.array([0., 0., yaw])

    node.subscribe(EvPose, cb_evpose)

    while not renderer.draw():
        node.spin_once()


if __name__ == "__main__":
    main()
