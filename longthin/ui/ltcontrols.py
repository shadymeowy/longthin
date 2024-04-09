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
        self.setLayout(layout)

        self.send_motor_button.clicked.connect(self.send_motor)
        self.send_setpoint_button.clicked.connect(self.send_setpoint)
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