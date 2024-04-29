import numpy as np

from .ltpacket import *


class ParkingEstimator:
    def __init__(self, node):
        self.node = node
        self.params = node.params
        self.config = node.config

        self.entry_xs0 = []
        self.entry_ys0 = []
        self.entry_xs1 = []
        self.entry_ys1 = []
        self.entry_xs2 = []
        self.entry_ys2 = []
        self.entry_xs3 = []
        self.entry_ys3 = []

        self.ekf_pos = np.array([0, 0])
        self.ekf_yaw = 0

        self.node.subscribe(EkfState, self.cb_ekf)
        self.node.subscribe(GoalVision, self.cb_goal_vision)

    def cb_ekf(self, packet):
        self.ekf_pos = np.array([packet.x, packet.y])
        self.ekf_yaw = packet.yaw

    def cb_goal_vision(self, packet):
        goal_x = packet.center_x
        goal_area = packet.area
        config = self.config

        area_w = config.renderer.area.width
        area_h = config.renderer.area.height
        distance = config.renderer.spot.length

        fx = config.camera.model.fx
        width = config.camera.model.width
        mean_x = goal_x * (0.5 * width)
        angle = np.arctan(mean_x / fx)
        yaw = np.deg2rad(self.ekf_yaw)
        angle = yaw + angle

        x = self.ekf_pos[0]
        y = self.ekf_pos[1]

        xp = None
        yp = None

        entry_xs0 = self.entry_xs0
        entry_ys0 = self.entry_ys0
        entry_xs1 = self.entry_xs1
        entry_ys1 = self.entry_ys1
        entry_xs2 = self.entry_xs2
        entry_ys2 = self.entry_ys2
        entry_xs3 = self.entry_xs3
        entry_ys3 = self.entry_ys3

        y0 = area_h / 2 + distance
        x0 = x + (area_h/2 - y + distance) / np.tan(angle)
        angle0 = (np.arctan2(y0 - y, x0 - x) - yaw) % (2 * np.pi)
        if (angle0 < np.pi / 2 or angle0 > 3 * np.pi / 2) and (-area_w < x0 < area_w):
            xp = x0
            yp = y0 - distance
            entry_xs0.append(xp)
            entry_ys0.append(yp)

        y1 = -area_h / 2 - distance
        x1 = x + (-area_h/2 - y - distance) / np.tan(angle)
        angle1 = (np.arctan2(y1 - y, x1 - x) - yaw) % (2 * np.pi)
        if (angle1 < np.pi / 2 or angle1 > 3 * np.pi / 2) and (-area_w < x1 < area_w):
            xp = x1
            yp = y1 + distance
            entry_xs1.append(xp)
            entry_ys1.append(yp)

        x2 = area_w / 2 + distance
        y2 = y + (area_w/2 - x + distance) * np.tan(angle)
        angle2 = (np.arctan2(y2 - y, x2 - x) - yaw) % (2 * np.pi)
        if (angle2 < np.pi / 2 or angle2 > 3 * np.pi / 2) and (-area_h < y2 < area_h):
            xp = x2 - distance
            yp = y2
            entry_xs2.append(xp)
            entry_ys2.append(yp)

        x3 = -area_w / 2 - distance
        y3 = y + (-area_w/2 - x - distance) * np.tan(angle)
        angle3 = (np.arctan2(y3 - y, x3 - x) - yaw) % (2 * np.pi)
        if (angle3 < np.pi / 2 or angle3 > 3 * np.pi / 2) and (-area_h < y3 < area_h):
            xp = x3 + distance
            yp = y3
            entry_xs3.append(xp)
            entry_ys3.append(yp)

        # TODO: parametrize the spot and approach positions
        mx = np.max([len(entry_xs0), len(entry_xs1), len(entry_xs2), len(entry_xs3)])
        lim = self.params.parking_estimator_xy_limit
        approach_d = self.params.parking_estimator_approach_d
        align_d = self.params.parking_estimator_alignment_d
        if len(entry_xs0) == mx:
            spot_x = np.mean(entry_xs0)
            spot_y = np.mean(entry_ys0)
            spot_x = max(-lim, min(lim, spot_x))
            approach_x = spot_x
            approach_y = spot_y - align_d
            spot_y -= approach_d
        elif len(entry_xs1) == mx:
            spot_x = np.mean(entry_xs1)
            spot_y = np.mean(entry_ys1)
            spot_x = max(-lim, min(lim, spot_x))
            approach_x = spot_x
            approach_y = spot_y + align_d
            spot_y += approach_d
        elif len(entry_xs2) == mx:
            spot_x = np.mean(entry_xs2)
            spot_y = np.mean(entry_ys2)
            spot_y = max(-lim, min(lim, spot_y))
            approach_x = spot_x - align_d
            approach_y = spot_y
            spot_x -= approach_d
        else:
            spot_x = np.mean(entry_xs3)
            spot_y = np.mean(entry_ys3)
            spot_y = max(-lim, min(lim, spot_y))
            approach_x = spot_x + align_d
            approach_y = spot_y
            spot_x += approach_d

        self.spot_pos = np.array([spot_x, spot_y])
        self.approach_pos = np.array([approach_x, approach_y])

    @property
    def measurement_count(self):
        return len(self.entry_xs0) + len(self.entry_xs1) + len(self.entry_xs2) + len(self.entry_xs3)

    def unsubscribe(self):
        self.node.unsubscribe(EkfState, self.cb_ekf)
        self.node.unsubscribe(GoalVision, self.cb_goal_vision)
