import time
import numpy as np

from .ltpacket import *


class ParkingController:
    def __init__(self, node):
        self.node = node
        self.params = node.params
        self.mean_x = 0
        self.enabled = False
        self._active = True
        self.last_time = None
        self.e_sum = 0
        self.e_last = 0

        self.node.subscribe(LaneVision, self.cb_lane_vision)

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False

    def control(self):
        t = time.time()
        if self.last_time is None:
            self.last_time = t
            return 0, 0
        dt = t - self.last_time

        e = self.mean_x
        self.e_sum += e * dt
        ki_limit = self.params.parking_ki_limit
        self.e_sum = np.clip(self.e_sum, -ki_limit, ki_limit)
        e_deriv = (e - self.e_last) / dt
        self.e_last = e
        u_w = (self.params.parking_kp * e
               + self.params.parking_ki * self.e_sum
               + self.params.parking_kd * e_deriv)
        u_v = 0.7
        u_l = u_v + u_w / 2
        u_r = u_v - u_w / 2

        if u_l < 0:
            u_r -= u_l
            u_l = 0
        elif u_r < 0:
            u_l -= u_r
            u_r = 0
        u_l = np.clip(u_l, 0, 1.0)
        u_r = np.clip(u_r, 0, 1.0)

        if not self._active:
            return 0, 0
        return u_l, u_r

    def cb_lane_vision(self, packet):
        mean_x = packet.mean_x
        min_y = packet.min_y
        if mean_x is not None:
            self.mean_x = mean_x
        if min_y is not None and min_y >= 0.83:
            self._active = False
        if min_y is not None and min_y < 0.83:
            self._active = True
        left, right = self.control()
        packet = Motor(left, right)
        self.node.publish(packet)
