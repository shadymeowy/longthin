import sys
from PySide6.QtCore import *
from PySide6.QtWidgets import *

from ..node import LTNode
from ..ltpacket import *
from ..ui import LTPFD


def main():
    app = QApplication(sys.argv)
    node = LTNode()

    timer = QTimer()
    timer.timeout.connect(node.spin_once)
    timer.start(1000 / 30)

    window = LTPFD(node)
    window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
