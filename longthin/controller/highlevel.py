import enum

from .controller_abc import ControllerABC
from .parking import ParkingController
from .vision import VisionController
from .dubins import DubinsController
from .goal import GoalController
from .manual import ManualController
from .setpoint import SetpointController
from .setpoint_pos import SetpointPosController


class ControllerMode(enum.Enum):
    MANUAL = 0
    VISION = 1
    SETPOINT = 2
    SETPOINT_POS = 3
    DUBINS = 4
    YAW = 5
    GOAL = 6
    PARKING = 7


mapo_mode_controller = {
    ControllerMode.MANUAL: ManualController,
    ControllerMode.VISION: VisionController,
    ControllerMode.SETPOINT: SetpointController,
    ControllerMode.SETPOINT_POS: SetpointPosController,
    ControllerMode.DUBINS: DubinsController,
    ControllerMode.GOAL: GoalController,
    ControllerMode.PARKING: ParkingController,
}


class HighLevelController(ControllerABC):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.controllers = {}
        for mode, controller in mapo_mode_controller.items():
            self.controllers[mode] = controller(self.node)
        self.mode = ControllerMode.MANUAL
        self.controller = self.controllers[self.mode]

    def set_mode(self, mode):
        if mode == self.mode:
            return
        prev_controller = self.controller
        prev_controller.enabled = False
        self.mode = mode
        self.controller = self.controllers[self.mode]
        self.controller.enabled = True

    def setpoint(self, *args, **kwargs):
        self.controllers[self.mode].setpoint(*args, **kwargs)

    def control(self):
        return self.controllers[self.mode].control()

    @property
    def is_reached(self):
        return self.controllers[self.mode].is_reached

    @is_reached.setter
    def is_reached(self, value):
        self.controllers[self.mode].is_reached = value