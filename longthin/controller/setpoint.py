from ..ltpacket import *
from .controller_abc import ControllerABC


class SetpointController(ControllerABC):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.vel = 0
        self.yaw = 0

    def setpoint(self, vel, yaw):
        self.vel = vel
        self.yaw = yaw

    def control(self):
        return Setpoint(self.vel, self.yaw)
