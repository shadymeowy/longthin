import yaml
from PySide6 import *
from PySide6.QtCore import *
from PySide6.QtGui import *
from PySide6.QtWidgets import *

from ..ltpacket import *


class ParamsUI(QWidget):
    def __init__(self, params, conn, parent=None):
        super(ParamsUI, self).__init__(parent)
        self.setWindowTitle('LTParameters')
        self.setGeometry(100, 100, 800, 600)
        self.conn = conn
        self.table = QTableWidget()
        self.table.setColumnCount(4)
        self.table.setHorizontalHeaderLabels(['ID', 'Name', 'Type', 'Value'])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(3, QHeaderView.Stretch)
        items = params.items()
        items = sorted(items, key=lambda x: x[0].name)
        for param, value in items:
            self.add_param(param.value, param.name, value, get_param_type(param).name)
        self.table.itemChanged.connect(self.changed)
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)
        self.layout.addWidget(self.table)
        self.save_button = QPushButton('Save')
        self.save_button.clicked.connect(self.save)
        self.layout.addWidget(self.save_button)
        self.load_button = QPushButton('Load')
        self.load_button.clicked.connect(self.load)
        self.layout.addWidget(self.load_button)

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
        param = LTParamType(id_)
        value = self.table.item(item.row(), 3).text()
        try:
            value = int(value)
        except ValueError:
            value = float(value)
        packet = setparam(param, value)
        print('packet', packet)
        self.conn.send(packet)

    def save(self):
        filename, _ = QFileDialog.getSaveFileName(self, 'Save File', '', 'YAML Files (*.yaml)')
        if filename:
            self.save_params(filename)
        else:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Critical)
            msg.setText('Filename not specified')
            msg.setWindowTitle('Error')
            msg.exec_()
            return

    def load(self):
        filename, _ = QFileDialog.getOpenFileName(self, 'Open File', '', 'YAML Files (*.yaml)')
        if filename:
            self.load_params(filename)
        else:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Critical)
            msg.setText('Filename not specified')
            msg.setWindowTitle('Error')
            msg.exec_()
            return

    def save_params(self, filename):
        # TODO: make it more type safe
        params = {}
        for row in range(self.table.rowCount()):
            name = self.table.item(row, 1).text()
            value = self.table.item(row, 3).text()
            params[name] = {'name': name, 'value': value}
        with open(filename, 'w') as f:
            yaml.dump(params, f)

    def load_params(self, filename):
        with open(filename, 'r') as f:
            params = yaml.load(f, Loader=yaml.FullLoader)
        for name, param in params.items():
            for row in range(self.table.rowCount()):
                if self.table.item(row, 1).text() == name:
                    self.table.item(row, 3).setText(str(param['value']))
                    break


def main():
    import sys
    ltzmq = LTZmq()
    app = QApplication(sys.argv)
    params = default_params()
    window = ParamsUI(params, ltzmq)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
