from abc import ABC, abstractmethod


class ControllerABC(ABC):
    def __init__(self, node):
        self.node = node
        self.params = node.params
        self.enabled = False
        self._is_reached = False

    @abstractmethod
    def control(self):
        pass

    @abstractmethod
    def setpoint(self, *args, **kwargs):
        pass

    @property
    def is_reached(self):
        return self._is_reached

    @is_reached.setter
    def is_reached(self, value):
        self._is_reached = value
