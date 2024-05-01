from ..ltpacket import *
from .vision import VisionController
from ..notify import Notify
import time


class ParkingController(VisionController):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.node.subscribe(LaneVision, self.cb_lane_vision)
        self.notify = Notify(self.node)
        self.last_time = time.time()

    def setpoint(self):
        pass

    def cb_lane_vision(self, packet):
        mean_x = packet.mean_x
        min_y = packet.min_y
        self.mean_x = mean_x
        if min_y is not None and min_y >= self.params.parking_visibility_limit:
            self.is_reached = True
        if min_y is not None and min_y < self.params.parking_visibility_limit:
            self.is_reached = False

        if not self.enabled:
            return
        # TODO: limit sending rate
        self.notify.beeping(1-min_y/self.params.parking_visibility_limit)
