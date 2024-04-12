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
    camera_rel_pose = node.config.camera.pose
    camera_rel_pose = Pose(
        [*camera_rel_pose.position[0:2], 0],
        [0, 0, camera_rel_pose.attitude[2]]
    )

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

        if ekf.ekf is None:
            return

        ekf_angle = np.rad2deg(ekf.angle) % 360
        est_pose_cam = Pose([ekf.x, ekf.y, 0], [0, 0, ekf_angle])
        est_pose = est_pose_cam.from_frame(camera_rel_pose.inv())

        ekf_state.x = est_pose.pos[0]
        ekf_state.y = est_pose.pos[1]
        ekf_state.yaw = ekf_angle
        node.publish(ekf_state)

    def cb_ev(packet):
        p = np.array([packet.x, packet.y])
        yaw = np.deg2rad(packet.yaw)
        ekf.set_ev_pose(p, yaw)
        md, ret = ekf.step()
        print(ret)

    def cb_ekfreset(packet):
        nonlocal ekf
        print("Resetting EKF")
        ekf = EKFAdapter()

    node.subscribe(Imu, cb_imu)
    node.subscribe(EvPose, cb_ev)
    node.subscribe(EkfReset, cb_ekfreset)

    node.spin()


if __name__ == "__main__":
    main()
