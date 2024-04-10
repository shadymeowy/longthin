import time
import numpy as np
from PySide6 import *
from PySide6.QtCore import *
from PySide6.QtGui import *
from PySide6.QtWidgets import *
from scipy.spatial.transform import Rotation

from ..ltpacket import *


class LTMap(QWidget):
    def __init__(self, node, parent=None):
        super(LTMap, self).__init__(parent)
        self.setWindowTitle('LTMap')
        self.zoom = 1
        self.yaw_ekf = 0
        self.pos_ekf = [0, 0]
        self.yaw_ev = 0
        self.pos_ev = [0, 0]
        self.last_ekf_time = 0

        self.node = node
        self.node.subscribe(EvPose, self.cb_ev)
        self.node.subscribe(EkfState, self.cb_ekf)

        self.mouse_press = False
        self.will_stop = False

        QApplication.instance().paletteChanged.connect(self.update_style)
        self.update_style()

    def update_style(self, palette=None):
        palette = palette or self.palette()
        self.dpi = min(self.logicalDpiX(), self.logicalDpiY())
        self.scale = (self.dpi / 96) * self.zoom
        z = self.zoom
        s = self.scale

        self.fg = palette.color(QPalette.WindowText)
        self.hg = palette.color(
            QPalette.Active, QPalette.Highlight)
        self.bg = palette.color(QPalette.Base)
        self.wg = palette.color(QPalette.Window)
        self.fg_n = palette.color(QPalette.Disabled, QPalette.WindowText)

        self.fg2 = QPen(self.fg, 2 * z)
        self.fg2.setJoinStyle(Qt.RoundJoin)
        self.fg2.setCapStyle(Qt.RoundCap)

        self.hg2 = QPen(self.hg, 2 * z)
        self.hg2.setJoinStyle(Qt.RoundJoin)
        self.hg2.setCapStyle(Qt.RoundCap)

        self.fg_n2 = QPen(self.fg_n, 2 * z)
        self.fg_n2.setJoinStyle(Qt.RoundJoin)
        self.fg_n2.setCapStyle(Qt.RoundCap)

        self.font16 = QFont("Arial", 16 * z)

    def paintEvent(self, e):
        dpi = min(self.logicalDpiX(), self.logicalDpiY())
        if dpi != self.dpi:
            self.update_style()

        self.painter = QPainter()
        self.painter.begin(self)
        self.painter.setRenderHints(QPainter.Antialiasing)
        self.painter.setRenderHints(QPainter.TextAntialiasing)
        self.painter.setPen(self.fg2)
        self.painter.setFont(self.font16)

        self.draw_background()
        self.draw_ev_pose()
        self.draw_ekf_pose()

        self.painter.end()

    def draw_background(self):
        w = self.width()
        h = self.height()
        wh = min(w, h)
        self.painter.save()
        self.painter.translate(w/2, h/2)
        self.painter.setPen(self.fg2)
        self.painter.setBrush(self.bg)
        self.painter.drawRect(-wh/2, -wh/2, wh, wh)
        self.painter.restore()

    def draw_ekf_pose(self):
        if time.time() - self.last_ekf_time > 0.5:
            return
        self.draw_pose(self.pos_ekf, self.yaw_ekf, self.fg2, self.fg, self.wg)

    def draw_ev_pose(self):
        self.draw_pose(self.pos_ev, self.yaw_ev, self.fg_n2, self.fg_n, Qt.NoBrush)

    def draw_pose(self, pos, yaw, pen, brush1, brush2):
        w = self.width()
        h = self.height()
        wh = min(w, h)
        config = self.node.config
        area_w = config.renderer.area.width
        area_h = config.renderer.area.height
        s = min(wh/area_w, wh/area_h)
        self.painter.save()
        self.painter.translate(w/2, h/2)
        self.painter.rotate(-90)
        self.painter.setPen(pen)
        self.painter.setBrush(brush1)

        chassis = config.renderer.chassis
        chassis_w = chassis.width * s
        chassis_l = chassis.length * s
        chassis_offset = chassis.offset * s

        x = pos[0] * s
        y = pos[1] * s
        self.painter.save()
        self.painter.translate(x, y)
        self.painter.rotate(yaw)
        self.painter.setBrush(brush2)
        self.painter.drawRect(-chassis_offset, -chassis_w/2, chassis_l, chassis_w)

        self.painter.setBrush(brush1)
        self.painter.drawEllipse(QPointF(0, 0), 5, 5)
        self.painter.drawLine(0, 0, 20, 0)
        self.painter.restore()
        self.painter.restore()

    def cb_ev(self, packet):
        self.yaw_ev = packet.yaw
        self.pos_ev = [packet.x, packet.y]
        self.update()

    def cb_ekf(self, packet):
        self.yaw_ekf = packet.yaw
        self.pos_ekf = [packet.x, packet.y]
        self.last_ekf_time = time.time()
        self.update()
