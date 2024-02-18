from PySide6 import *
from PySide6.QtCore import *
from PySide6.QtGui import *
from PySide6.QtWidgets import *

from ..ltpacket import *


class ParamsUI(QWidget):
    def __init__(self, params, conn, parent=None):
        super(ParamsUI, self).__init__(parent)
        self.setWindowTitle('Parameters')
        self.setGeometry(100, 100, 800, 600)
        self.conn = conn
        self.table = QTableWidget()
        self.table.setColumnCount(4)
        self.table.setHorizontalHeaderLabels(['ID', 'Name', 'Type', 'Value'])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(3, QHeaderView.Stretch)
        for param, value in params.items():
            self.add_param(param.value, param.name, value, get_param_type(param).name)
        self.table.itemChanged.connect(self.changed)
        self.layout = QVBoxLayout()
        self.layout.addWidget(self.table)
        self.setLayout(self.layout)

    def add_param(self, id_, name, value, type):
        index = self.table.rowCount()
        self.table.insertRow(index)
        item = QTableWidgetItem(hex(id_))
        item.setFlags(item.flags() & ~Qt.ItemIsEditable)
        self.table.setItem(index, 0, item)
        item = QTableWidgetItem(name)
        item.setFlags(item.flags() & ~Qt.ItemIsEditable)
        self.table.setItem(index, 1, item)
        item = QTableWidgetItem(str(type))
        item.setFlags(item.flags() & ~Qt.ItemIsEditable)
        self.table.setItem(index, 2, item)
        item = QTableWidgetItem(str(value))
        self.table.setItem(index, 3, item)

    def changed(self, item):
        id_ = int(self.table.item(item.row(), 0).text(), 16)
        param = LTParams(id_)
        value = float(self.table.item(item.row(), 3).text())
        packet = setparam(param, value)
        print('packet', packet)
        self.conn.send(packet)


def main():
    import sys
    ltzmq = LTZmq(5555, 5556, server=False)
    app = QApplication(sys.argv)
    params = default_params()
    window = ParamsUI(params, ltzmq)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
