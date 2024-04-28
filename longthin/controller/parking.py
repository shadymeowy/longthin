from ..ltpacket import *
from .vision import VisionController


class ParkingController(VisionController):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.node.subscribe(LaneVision, self.cb_lane_vision)

    def setpoint(self):
        pass

    def cb_lane_vision(self, packet):
        mean_x = packet.mean_x
        min_y = packet.min_y
        self.mean_x = mean_x
        # TODO: this needs to be parameterized
        if min_y is not None and min_y >= 0.83:
            self.is_reached = True
        if min_y is not None and min_y < 0.83:
            self.is_reached = False