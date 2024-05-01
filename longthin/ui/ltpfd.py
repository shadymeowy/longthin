import numpy as np
from PySide6 import *
from PySide6.QtCore import *
from PySide6.QtGui import *
from PySide6.QtWidgets import *
from scipy.spatial.transform import Rotation

from ..ltpacket import *
from QPrimaryFlightDisplay import QPrimaryFlightDisplay


class LTPFD(QPrimaryFlightDisplay):
    def __init__(self, node, parent=None):
        super(LTPFD, self).__init__(parent)
        self.node = node
        self.node.subscribe(Imu, self.cb_imu)
        self.node.subscribe(AdcRead, self.cb_adc)
        self.setWindowTitle("Longthin PFD")

        self.battery = 100
        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(1000 / 30)

    def cb_imu(self, packet):
        rot = Rotation.from_quat([packet.qx, packet.qy, packet.qz, packet.qw])
        euler = rot.as_euler('xyz', degrees=False)
        self.roll = euler[0]
        self.pitch = euler[1]
        self.heading = np.rad2deg(euler[2])

    def cb_adc(self, packet):
        # TODO: Add this to config
        if packet.id == 1:
            value = packet.value / 8.4 * 100
            self.battery = min(self.battery, value)
        elif packet.id == 0:
            value = packet.value / 16.8 * 100
            self.battery = min(self.battery, value)
        else:
            raise ValueError("Invalid ADC ID")
