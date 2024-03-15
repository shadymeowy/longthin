import sys
import time
from PySide6.QtWidgets import *
from PySide6.QtGui import *
from PySide6.QtCore import *
from QPrimaryFlightDisplay import QPrimaryFlightDisplay

from ..ltpacket import *
from ..config import load_config


class LTApp(QDialog):
    def __init__(self, conn, config):
        super().__init__()
        self.conn = conn
        self.config = config

        self.setWindowTitle("Longthin PFD")
        self.setGeometry(100, 100, 800, 600)
        self.layout = QGridLayout()
        self.setLayout(self.layout)
        self._x = 0
        self._y = 0

        self.button_led = self.append_widget(QPushButton("LED"))
        self.button_led.clicked.connect(self.led)
        self.led_state = False
        self.append_newline()

        self.input_yaw = self.append_lineedit("Yaw", "0")
        self.input_vel = self.append_lineedit("Vel", "0")
        self.button_setpoint = self.append_widget(QPushButton("Setpoint"))
        self.button_setpoint.clicked.connect(self.setpoint)
        self.append_newline()

        self.input_motor_left = self.append_lineedit("Motor Left", "0")
        self.input_motor_right = self.append_lineedit("Motor Right", "0")
        self.button_motor = self.append_widget(QPushButton("Motor"))
        self.button_motor.clicked.connect(self.motor_setpoint)

        self.append_newline()
        self.pfd = self.append_widget(QPrimaryFlightDisplay(), 8, 10)
        self.pfd.zoom = 1

        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(1000 / 120)

        self.x = 0
        self.y = 0

    def setpoint(self):
        packet = Setpoint(0, 0)
        packet.vel = float(self.input_vel.text())
        packet.yaw = float(self.input_yaw.text())
        self.conn.send(packet)

    def motor_setpoint(self):
        packet = Motor(0, 0)
        packet.left = float(self.input_motor_left.text())
        packet.right = float(self.input_motor_right.text())
        self.conn.send(packet)

    def keyPressEvent(self, event):
        super().keyPressEvent(event)
        if event.isAutoRepeat():
            return
        key = event.key()
        if key == Qt.Key.Key_W:
            self.x = 1
        elif key == Qt.Key.Key_S:
            self.x = -1
        elif key == Qt.Key.Key_A:
            self.y = -1
        elif key == Qt.Key.Key_D:
            self.y = 1
        else:
            return
        self.update_motor()

    def keyReleaseEvent(self, event):
        super().keyReleaseEvent(event)
        if event.isAutoRepeat():
            return
        key = event.key()
        if key == Qt.Key.Key_W:
            self.x = 0
        elif key == Qt.Key.Key_S:
            self.x = 0
        elif key == Qt.Key.Key_A:
            self.y = 0
        elif key == Qt.Key.Key_D:
            self.y = 0
        else:
            return
        self.update_motor()

    def update_motor(self):
        x = self.x
        y = self.y
        conf = self.config.manual_control
        f = conf.forward_backward
        l = conf.left_right
        m1 = conf.mixed_1
        m2 = conf.mixed_2
        
        if x == 0 and y == 0:
            self.set_motor(0, 0)
        elif x == 0:
            # turn left or right without moving forward
            self.set_motor(l * y, -l * y)
        elif y == 0:
            # go forward or backward without turning
            self.set_motor(x * f, x * f)
        elif x > 0:
            if y > 0:
                # go forward and turn right
                self.set_motor(m1, m2)
            else:
                # go forward and turn left
                self.set_motor(m2, m1)
        else:
            if y > 0:
                # go backward and turn left
                self.set_motor(-m2, -m1)
            else:
                # go backward and turn right
                self.set_motor(-m1, -m2)

    def set_motor(self, left, right):
        packet = Motor(left, right)
        self.conn.send(packet)

    def append_widget(self, widget, w=1, h=1):
        self.layout.addWidget(widget, self._y, self._x, h, w)
        self._x += 1
        return widget

    def append_newline(self):
        self._x = 0
        self._y += 1

    def append_lineedit(self, label, default):
        self.append_widget(QLabel(label))
        return self.append_widget(QLineEdit(default))

    def led(self):
        packet = Led(0, self.led_state)
        packet.state = 1 - self.led_state
        self.led_state = packet.state
        self.conn.send(packet)

    def update(self):
        while True:
            packet = self.conn.read()
            pfd = self.pfd
            if packet is None:
                return
            if isinstance(packet, Imu):
                pfd.roll = packet.roll * 3.1415926 / 180
                pfd.pitch = packet.pitch * 3.1415926 / 180
                pfd.heading = -packet.yaw
                pfd.update()


def main():
    app = QApplication(sys.argv)
    conn = LTZmq(5555, 5556, server=False)
    config = load_config("default.yaml")
    window = LTApp(conn, config)
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
