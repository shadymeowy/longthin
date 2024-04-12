import numpy as np
import argparse

from ..node import LTNode
from ..graphics import LTRenderer
from ..ltpacket import *


def main():
    parser = argparse.ArgumentParser(description='A visualization of the robot')
    parser.add_argument('--mode', default='sim', choices=['sim', 'ekf', 'ev'], help='Mode of operation')
    args = parser.parse_args()

    node = LTNode()
    renderer = LTRenderer(node.config)

    def callback(packet):
        x, y, yaw = packet.x, packet.y, packet.yaw
        renderer.vehicle_pose.pos = np.array([x, y, 0.])
        renderer.vehicle_pose.att = np.array([0., 0., yaw])

    if args.mode == 'sim':
        node.subscribe(SimState, callback)
    elif args.mode == 'ekf':
        node.subscribe(EkfState, callback)
    elif args.mode == 'ev':
        node.subscribe(EvPose, callback)
    else:
        raise ValueError('Invalid mode')

    while not renderer.draw():
        node.spin_once()


if __name__ == "__main__":
    main()
