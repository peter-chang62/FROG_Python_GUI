import thorlabs_apt as apt
import numpy as np
import threading
import time
from PyQt5 import QtCore, QtGui, QtWidgets


class Signal(QtCore.QObject):
    progress = QtCore.pyqtSignal(float)


class motor_update(threading.Thread):
    def __init__(self, motor, box):
        super().__init__()

        motor: apt.Motor
        self.motor = motor

        self.signal = Signal()
        self.box = box

    def move_to_pos(self, pos_mm):
        self.motor.position = pos_mm

    def run(self):
        self.signal.progress.connect(self.box.slot)
        while self.motor.is_in_motion:
            pos = self.motor.position
            print(pos)
            self.signal.progress.emit(pos)
            time.sleep(.005)


class Box(QtWidgets.QLineEdit):
    def __init__(self):
        super().__init__()
        self.show()

    def slot(self, pos):
        self.setText(str(pos))


serial = apt.list_available_devices()[0][1]
m = apt.Motor(serial)
param = list(m.get_velocity_parameters())
param[-1] = 1.
m.set_velocity_parameters(*param)
# m.move_home(blocking=True)

app = QtWidgets.QApplication([])
window = Box()

Motor = motor_update(m, window)
Motor.move_to_pos(10)
Motor.start()
app.exec()
