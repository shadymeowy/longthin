from ..ltpacket import *
from .controller_abc import ControllerABC


class SetpointPosController(ControllerABC):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.x = 0
        self.y = 0

    def setpoint(self, x, y):
        self.x = x
        self.y = y

    def control(self):
        return SetpointPos(self.x, self.y)