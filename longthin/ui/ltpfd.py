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
        self.setWindowTitle("Longthin PFD")

        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(1000 / 30)

    def cb_imu(self, packet):
        rot = Rotation.from_quat([packet.qx, packet.qy, packet.qz, packet.qw])
        euler = rot.as_euler('xyz', degrees=False)
        self.roll = euler[0]
        self.pitch = euler[1]
        self.heading = np.rad2deg(euler[2])