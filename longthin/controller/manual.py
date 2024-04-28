from ..ltpacket import *
from .controller_abc import ControllerABC


class ManualController(ControllerABC):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.left = 0
        self.right = 0

    def setpoint(self, left, right):
        self.left = left
        self.right = right

    def control(self):
        if not self.enabled:
            return Motor(0, 0)
        return Motor(self.left, self.right)
