from ..ltpacket import *
from .vision import VisionController


class GoalController(VisionController):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.node.subscribe(GoalVision, self.cb_goal_vision)

    def setpoint(self):
        pass

    def cb_goal_vision(self, packet):
        self.is_parking = True
        self.mean_x = packet.center_x
        self.goal_area = packet.area
