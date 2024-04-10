import time
import numpy as np
from PySide6 import *
from PySide6.QtCore import *
from PySide6.QtGui import *
from PySide6.QtWidgets import *
from scipy.spatial.transform import Rotation

from ..ltpacket import *


class LTJoystick(QWidget):
    def __init__(self, node, parent=None):
        super(LTJoystick, self).__init__(parent)
        self.setWindowTitle('LTJoystick')
        self.zoom = 1
        self.current_imu = 0
        self.current_ekf = 0
        self.last_ekf_time = 0
        self.target_heading = 0

        self.node = node
        self.node.subscribe(Imu, self.cb_imu)
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
        self.draw_imu()
        self.draw_ekf()
        self.draw_target()

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

    def draw_imu(self):
        self.painter.setPen(self.fg_n2)
        self.draw_arrow(self.current_imu)

    def draw_ekf(self):
        # TODO: Add a timeout parameter for ekf exclusive
        timeout = self.node.params.controller_timeout / 1e6
        if time.time() - self.last_ekf_time > timeout:
            return
        self.painter.setPen(self.fg2)
        self.draw_arrow(self.current_ekf)

    def draw_target(self):
        self.painter.setPen(self.hg2)
        self.draw_arrow2(self.target_heading)

    def draw_arrow(self, heading):
        self.painter.save()
        h = self.height()
        w = self.width()
        wh = min(w, h)
        self.painter.translate(w/2, h/2)
        self.painter.rotate(heading)
        self.painter.drawLine(0, 0, 0, -wh*0.40)
        self.painter.drawLine(0, -wh*0.40, -wh*0.01, -wh*0.38)
        self.painter.drawLine(0, -wh*0.40, wh*0.01, -wh*0.38)
        self.painter.restore()

    def draw_arrow2(self, heading):
        self.painter.save()
        h = self.height()
        w = self.width()
        wh = min(w, h)
        self.painter.translate(w/2, h/2)
        self.painter.rotate(heading)
        self.painter.drawLine(0, 0, 0, -wh*0.37)
        self.painter.drawLine(0, -wh*0.37, -wh*0.01, -wh*0.38)
        self.painter.drawLine(-wh*0.01, -wh*0.38, -wh*0.01, -wh*0.40)
        self.painter.drawLine(wh*0.01, -wh*0.40, wh*0.01, -wh*0.38)
        self.painter.drawLine(wh*0.01, -wh*0.38, 0, -wh*0.37)
        self.painter.restore()

    def cb_imu(self, packet):
        rot = Rotation.from_quat([packet.qx, packet.qy, packet.qz, packet.qw])
        euler = rot.as_euler('xyz', degrees=True)
        self.current_imu = euler[2]
        self.send_control()
        self.update()

    def cb_ekf(self, packet):
        self.current_ekf = packet.yaw
        self.last_ekf_time = time.time()
        self.update()

    def update_target(self):
        x = self.mouse_pos.x()
        y = self.mouse_pos.y()
        w = self.width()
        h = self.height()
        wh = min(w, h)
        x = 4 * (x - w/2) / wh
        y = 4 * (h/2 - y) / wh
        self.target_heading = np.arctan2(x, y) * 180 / np.pi
        self.update()

    def send_control(self):
        if self.mouse_press:
            packet = Setpoint(1.0, self.target_heading)
            self.node.publish(packet)
        if self.will_stop:
            packet = Motor(0, 0)
            self.node.publish(packet)
            self.will_stop = False

    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            self.mouse_press = True
            self.mouse_pos = e.pos()
            self.update_target()
        super().mousePressEvent(e)

    def mouseReleaseEvent(self, e):
        if e.button() == Qt.LeftButton:
            self.mouse_press = False
            self.will_stop = True
        super().mouseReleaseEvent(e)

    def mouseMoveEvent(self, e):
        if self.mouse_press:
            self.mouse_pos = e.pos()
            self.update_target()
        super().mouseMoveEvent(e)
