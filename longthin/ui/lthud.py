from PySide6 import *
from PySide6.QtCore import *
from PySide6.QtGui import *
from PySide6.QtWidgets import *

from ..ltpacket import *

to_show = [
    (MotorOutput, 'left'),
    (MotorOutput, 'right'),
    (Motor, 'left'),
    (Motor, 'right'),
    (Setpoint, 'vel'),
    (Setpoint, 'yaw'),
    (ControlDebug, 'current_yaw'),
    (ControlDebug, 'desired_yaw'),
    (SimState, 'x'),
    (SimState, 'y'),
    (SimState, 'theta'),
    (EvPose, 'x'),
    (EvPose, 'y'),
    (EvPose, 'yaw')]


class LTHUD(QWidget):
    def __init__(self, node, parent=None):
        super(LTHUD, self).__init__(parent)
        self.setWindowTitle('LTHUD')
        self.setGeometry(100, 100, 800, 600)
        self.node = node

        self.table = QTableWidget(len(to_show), 2)
        self.table.setHorizontalHeaderLabels(['Property', 'Value'])
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.horizontalHeader().setVisible(False)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.setLayout(QVBoxLayout())
        self.layout().addWidget(self.table)
        self.props = {}
        self.label_values = {}

        for row, (typ, prop) in enumerate(to_show):
            self.node.subscribe(typ, self.handle_packet)
            enum = type_map_rev[typ]
            name = str(enum.name).lower() + '.' + prop
            self.table.setItem(row, 0, QTableWidgetItem(name))
            label_value = QTableWidgetItem()
            self.table.setItem(row, 1, label_value)
            self.label_values[row] = ""
            if typ not in self.props:
                self.props[typ] = []
            self.props[typ].append((prop, row))

        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(1000 / 15)

    def handle_packet(self, packet):
        typ = type(packet)
        for prop, row in self.props[typ]:
            value = getattr(packet, prop)
            self.label_values[row] = "{:.2f}".format(value)

    def update(self):
        for row, value in self.label_values.items():
            self.table.item(row, 1).setText(value)
