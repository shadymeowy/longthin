from longthin import *

import numpy as np


class DubinsController:
    def __init__(self, node):
        self.node = node

        self.target = None
        self.gen_path = None
        self.is_limited = True
        self.target_reached = False
        self.enabled = False

        # TODO: parameterize these
        self.R = 0.2
        self.limit_angle = 60
        self.vehicle_points = np.array([
            [-0.0, 0.0, 0.],
            [-0.0, -0.0, 0.],
            [0.6, 0.0, 0.],
            [0.6, -0.0, 0.]
        ])

        self.node.subscribe(EkfState, self.cb_ekf)

    def set_target(self, target_x, target_y, circle=False):
        self.target = np.array([target_x, target_y, 0], dtype=np.float32)
        self.gen_path = None
        self.is_limited = True
        self.target_reached = False
        if circle:
            self.methods = [path_rsrc, path_rslc]
        else:
            self.methods = [path_rs]

    def cb_ekf(self, packet):
        self.ekf_pos = np.array([packet.x, packet.y, 0])
        self.ekf_yaw = packet.yaw

        if self.target is None:
            return

        if self.gen_path is None:
            print('Generating path')
            print(self.ekf_pos,
                  self.target,
                  self.ekf_yaw,
                  0,
                  self.R,
                  self.vehicle_points,
                  self.methods)
            self.gen_path = dubins(
                self.ekf_pos,
                self.target,
                self.ekf_yaw,
                0,
                R=self.R,
                vps=self.vehicle_points,
                methods=self.methods)
            self.target_p = self.gen_path[1].q2
            self.target_v = self.gen_path[1].u2
            self.target_inf = self.gen_path[1].q2 + 0.5*self.gen_path[1].u2
            self.is_right = self.gen_path[0].n[2] >= 0
            return

        delta_inf_p = self.target_inf - self.ekf_pos
        self.angle_inf = np.arctan2(delta_inf_p[1], delta_inf_p[0])
        self.angle_inf = np.rad2deg(self.angle_inf)

        if self.is_right or not self.is_limited:
            angle_diff = +((self.angle_inf - self.ekf_yaw) % 360)
        else:
            angle_diff = -((self.ekf_yaw - self.angle_inf) % 360)

        if not self.is_limited and angle_diff > 180:
            angle_diff -= 360

        if np.abs(angle_diff) > self.limit_angle:
            angle_diff = np.sign(angle_diff) * self.limit_angle
        else:
            self.is_limited = False

        d = np.dot(self.target_p - self.ekf_pos, self.target_v)

        if d > 0:
            packet = Setpoint(1.0, self.ekf_yaw + angle_diff)
        else:
            self.target_reached = True
            packet = Motor(0, 0)

        if self.enabled:
            self.node.publish(packet)
