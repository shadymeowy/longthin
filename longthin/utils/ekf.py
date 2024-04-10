import argparse
import numpy as np
from scipy.spatial.transform import Rotation

from ..node import LTNode
from ..ltpacket import *
from ..filter.ekf import EKFAdapter
from ..geometry import Pose


def main():
    parser = argparse.ArgumentParser(description='A packet echo client')
    args = parser.parse_args()

    node = LTNode()
    ekf = EKFAdapter()
    ekf_state = EkfState(0, 0, 0)

    def cb_imu(packet):
        dvel = np.array([packet.dvx, packet.dvy])
        rot = Rotation.from_quat([packet.qx, packet.qy, packet.qz, packet.qw])
        euler = rot.as_euler('xyz', degrees=False)
        angle = euler[2]
        dt = packet.dt
        ekf.set_imu_dvel(dvel, angle, dt)
        ekf.step()

        ekf_state.x = ekf.x
        ekf_state.y = ekf.y
        ekf_state.yaw = np.rad2deg(ekf.angle) % 360
        node.publish(ekf_state)

    def cb_ev(packet):
        p = np.array([packet.x, packet.y])
        yaw = np.deg2rad(packet.yaw)
        ekf.set_ev_pose(p, yaw)
        md, ret = ekf.step()
        print(ret)

    node.subscribe(Imu, cb_imu)
    node.subscribe(EvPose, cb_ev)

    node.spin()


if __name__ == "__main__":
    main()
