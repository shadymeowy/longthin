import numpy as np
from PySide6 import *
from PySide6.QtCore import *
from PySide6.QtGui import *
from PySide6.QtWidgets import *
from scipy.spatial.transform import Rotation

from ..ltpacket import *
from . import LTJoystick
from . import LTPFD
from . import LTParamsUI


class LTGCS(QWidget):
    def __init__(self, node, parent=None):
        super(LTGCS, self).__init__(parent)
        self.setWindowTitle('LTGCS')
        self.setGeometry(100, 100, 800, 600)
        self.node = node

        self.tabs = QTabWidget()
        self.params_ui = LTParamsUI(node)
        self.pfd = LTPFD(node)
        self.joystick = LTJoystick(node)

        self.tabs.addTab(self.pfd, 'PFD')
        self.tabs.addTab(self.joystick, 'Joystick')
        self.tabs.addTab(self.params_ui, 'Params')

        layout = QVBoxLayout()
        layout.addWidget(self.tabs)
        self.setLayout(layout)
