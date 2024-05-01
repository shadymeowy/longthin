import numpy as np
from PySide6 import *
from PySide6.QtCore import *
from PySide6.QtGui import *
from PySide6.QtWidgets import *
from scipy.spatial.transform import Rotation

from ..ltpacket import *


class LTControls(QWidget):
    def __init__(self, node, parent=None):
        super(LTControls, self).__init__(parent)
        self.setWindowTitle('LTJoystick')
        self.node = node

        self.timer = QTimer()

        layout = QVBoxLayout(self)

        motor_layout = QHBoxLayout()
        self.left_motor_input = QLineEdit()
        self.left_motor_input.setPlaceholderText("Left Motor")
        self.right_motor_input = QLineEdit()
        self.right_motor_input.setPlaceholderText("Right Motor")
        self.send_motor_button = QPushButton("Send Motor")
        motor_layout.addWidget(self.left_motor_input)
        motor_layout.addWidget(self.right_motor_input)
        motor_layout.addWidget(self.send_motor_button)
        layout.addLayout(motor_layout)

        setpoint_layout = QHBoxLayout()
        self.desired_velocity_input = QLineEdit()
        self.desired_velocity_input.setPlaceholderText("Desired Velocity")
        self.desired_heading_input = QLineEdit()
        self.desired_heading_input.setPlaceholderText("Desired Heading")
        self.send_setpoint_button = QPushButton("Send Setpoint")
        setpoint_layout.addWidget(self.desired_velocity_input)
        setpoint_layout.addWidget(self.desired_heading_input)
        setpoint_layout.addWidget(self.send_setpoint_button)
        layout.addLayout(setpoint_layout)

        self.stop_movement_button = QPushButton("Stop Movement")
        layout.addWidget(self.stop_movement_button)

        self.toggle_led_button = QPushButton("Toggle LEDs")
        layout.addWidget(self.toggle_led_button)

        self.send_parking_button = QPushButton("Park")
        layout.addWidget(self.send_parking_button)

        self.send_ekfreset_button = QPushButton("EKF Reset")
        layout.addWidget(self.send_ekfreset_button)

        self.send_reboot_button = QPushButton("Reboot")
        layout.addWidget(self.send_reboot_button)
        self.setLayout(layout)

        self.send_motor_button.clicked.connect(self.send_motor)
        self.send_setpoint_button.clicked.connect(self.send_setpoint)
        self.send_parking_button.clicked.connect(self.send_parking)
        self.send_ekfreset_button.clicked.connect(self.send_ekfreset)
        self.send_reboot_button.clicked.connect(self.send_reboot)
        self.stop_movement_button.clicked.connect(self.stop_movement)
        self.toggle_led_button.clicked.connect(self.toggle_leds)

        self.led = Led(0, 0)
        self.will_stop = False
        self.packet = None

        self.timer.timeout.connect(self.cb_timer)
        self.timer.start(1000 / 30)

    def send_motor(self):
        left_motor = float(self.left_motor_input.text())
        right_motor = float(self.right_motor_input.text())
        self.packet = Motor(left_motor, right_motor)

    def send_setpoint(self):
        vel = float(self.desired_velocity_input.text())
        theta = float(self.desired_heading_input.text())
        self.packet = Setpoint(vel, theta)

    def send_parking(self):
        self.node.publish(ButtonState(0, 1))

    def send_ekfreset(self):
        self.node.publish(EkfReset(0))

    def send_reboot(self):
        self.node.publish(Reboot(0))

    def stop_movement(self):
        self.packet = None
        self.will_stop = True

    def toggle_leds(self):
        self.led.state = not self.led.state
        self.node.publish(self.led)

    def cb_timer(self):
        if self.packet is not None:
            self.node.publish(self.packet)
        if self.will_stop:
            self.node.publish(Motor(0, 0))
            self.will_stop = False
