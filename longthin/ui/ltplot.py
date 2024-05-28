import time
import pyqtgraph as pg
from PySide6 import *
from PySide6.QtCore import *
from PySide6.QtGui import *
from PySide6.QtWidgets import *

from ..node import LTNode
from ..ltpacket import *


class LTPlot(QWidget):
    def __init__(self, node, parent=None):
        super(LTPlot, self).__init__(parent)
        self.setWindowTitle('LTPlot')
        self.node = node

        self.layout = QVBoxLayout(self)
        self.setLayout(self.layout)

        grid_layout = QGridLayout()

        self.label_typs = QLabel('Types:')
        self.text_typs = QLineEdit()
        self.text_typs.setPlaceholderText('Packet and properties to plot')
        self.text_typs.setText('ev_pose.x,ekf_state.x ev_pose.y,ekf_state.y motor_output.left,motor_output.right control_debug.desired_yaw,control_debug.current_yaw')
        self.text_typs.returnPressed.connect(self.init_plot)

        self.label_duration = QLabel('Duration:')
        self.text_duration = QLineEdit()
        self.text_duration.setPlaceholderText('Max duration to plot')
        self.text_duration.setText('10')
        self.text_duration.returnPressed.connect(self.update_duration)
        self.duration = 10

        self.plotting = True
        self.plot_button = QPushButton('Pause')
        self.plot_button.setCheckable(True)
        self.plot_button.setChecked(True)
        self.plot_button.clicked.connect(self.cb_plot_button)
        grid_layout.addWidget(self.plot_button, 0, 2)

        self.reset_button = QPushButton('Reset')
        self.reset_button.clicked.connect(self.init_plot)
        grid_layout.addWidget(self.reset_button, 1, 2)

        grid_layout.addWidget(self.label_typs, 0, 0)
        grid_layout.addWidget(self.text_typs, 0, 1)
        grid_layout.addWidget(self.label_duration, 1, 0)
        grid_layout.addWidget(self.text_duration, 1, 1)
        self.layout.addLayout(grid_layout)

        # add a empty widget to hold the plot
        self.plot_widget = QWidget()
        self.plot_widget.setLayout(QVBoxLayout())
        self.layout.addWidget(self.plot_widget)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(1000 / 15)
        self.start_time = time.time()

        self.graphic_widget = None
        self.init_plot()

    def init_plot(self):
        if self.graphic_widget is not None:
            self.plot_widget.layout().removeWidget(self.graphic_widget)
            self.graphic_widget.close()
            self.graphic_widget.deleteLater()
            self.graphic_widget = None

        if self.text_typs.text() == '':
            return

        self.graphic_widget = pg.GraphicsLayoutWidget()
        self.plot_widget.layout().addWidget(self.graphic_widget)

        self.value_dict = {}
        self.packets = []
        # we should find a better way to store those
        for i, ps in enumerate(self.text_typs.text().split()):
            for packet in ps.split(','):
                typ, prop = packet.split('.')
                typ = LTPacketType[typ.upper()]
                prop = prop.lower()
                self.value_dict[(typ, prop)] = [[], [], None, i]
                self.packets.append(typ)
        self.packets = set(self.packets)

        self.plots = []
        for _ in range(len(self.text_typs.text().split())):
            plot = self.graphic_widget.addPlot()
            self.plots.append(plot)
            plot.showGrid(x=True, y=True)
            plot.setLabel('left', 'Value')
            plot.setLabel('bottom', 'Time (s)')
            plot.addLegend()
            self.graphic_widget.nextRow()

        for i, (k, v) in enumerate(self.value_dict.items()):
            name = LTPacketType(k[0]).name + '.' + k[1]
            v[2] = self.plots[v[3]].plot(pen=(i, len(self.value_dict)), name=name)

        for packet in self.packets:
            self.node.subscribe(packet.to_type(), self.update_value)

    def update_duration(self):
        self.duration = float(self.text_duration.text())

    def update_plot(self):
        if not self.isVisible() or self.graphic_widget is None:
            return
        if not self.plotting:
            return
        for (typ, prop), (ts, xs, curve, _) in self.value_dict.items():
            curve.setData(ts, xs)

    def update_value(self, packet):
        # worst code ever, but it works
        if self.graphic_widget is None:
            return
        t = time.time() - self.start_time
        if packet.type in self.packets:
            for (typ, prop), (ts, xs, curve, _) in self.value_dict.items():
                if packet.type != typ:
                    continue
                ts.append(t)
                xs.append(getattr(packet, prop))

        if self.duration is not None:
            for (typ, prop), (ts, xs, curve, _) in self.value_dict.items():
                while ts and ts[0] < t - self.duration:
                    ts.pop(0)
                    xs.pop(0)

    def cb_plot_button(self):
        if self.plot_button.isChecked():
            self.plot_button.setText('Pause')
        else:
            self.plot_button.setText('Play')
        self.plotting = self.plot_button.isChecked()
        self.update_plot()
