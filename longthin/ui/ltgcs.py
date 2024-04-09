import numpy as np
import os
from PySide6 import *
from PySide6.QtCore import *
from PySide6.QtGui import *
from PySide6.QtWidgets import *
from scipy.spatial.transform import Rotation

from ..ltpacket import *
from . import LTJoystick
from . import LTPFD
from . import LTParamsUI


# Using QDockWidget to create a GCS window
class LTGCS(QMainWindow):
    def __init__(self, node, parent=None):
        super(LTGCS, self).__init__(parent)
        self.setWindowTitle('LTGCS')
        self.setGeometry(100, 100, 800, 600)
        self.node = node

        # Empty central widget
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        features = (QDockWidget.DockWidgetMovable |
                    QDockWidget.DockWidgetFloatable)
        
        os.environ["NO_GL"] = "1"

        self.joystick_dock = QDockWidget('Joystick', self)
        self.joystick = LTJoystick(self.node)
        self.joystick_dock.setWidget(self.joystick)
        self.addDockWidget(Qt.LeftDockWidgetArea, self.joystick_dock)
        self.joystick_dock.setAllowedAreas(Qt.AllDockWidgetAreas)
        self.joystick_dock.setFeatures(features)

        self.pfd_dock = QDockWidget('PFD', self)
        self.pfd = LTPFD(self.node)
        self.pfd.zoom = 0.5
        self.pfd.update_style()
        self.pfd_dock.setWidget(self.pfd)
        self.addDockWidget(Qt.LeftDockWidgetArea, self.pfd_dock)
        self.pfd_dock.setAllowedAreas(Qt.AllDockWidgetAreas)
        self.pfd_dock.setFeatures(features)

        self.params_dock = QDockWidget('Parameters', self)
        self.params = LTParamsUI(self.node)
        self.params_dock.setWidget(self.params)
        self.addDockWidget(Qt.RightDockWidgetArea, self.params_dock)
        self.params_dock.setAllowedAreas(Qt.AllDockWidgetAreas)
        self.params_dock.setFeatures(features)

        # set size initial of right dock
        self.resizeDocks([self.params_dock], [350], Qt.Horizontal)
        self.resizeDocks([self.joystick_dock, self.pfd_dock], [350, 350], Qt.Horizontal)
        self.resizeDocks([self.joystick_dock, self.pfd_dock], [350, 350], Qt.Vertical)

        # set fixed size to pfd dock
        self.pfd_dock.setMaximumWidth(350)
        self.pfd_dock.setMinimumWidth(350)
        self.pfd_dock.setMaximumHeight(350)
        self.pfd_dock.setMinimumHeight(350)

        # set maximum size to joystick dock
        self.joystick_dock.setMaximumHeight(500)
        self.joystick_dock.setMaximumWidth(500)