"""The emulator class is made in the hopes that I just have to debug
utilities.py when I first connect to hardware. """

import numpy as np
import time
import PyQt5.QtCore as qtc
import scipy.constants as sc

pool = qtc.QThreadPool.globalInstance()

# global variables
# step_size_mm = 30e-9 * 1e3  # 30 nm
step_size_mm = 7.49481145e-09 * 1e3 / 2
sleep_time = .00005  # 1ms


class Motor:
    def __init__(self):
        # this is arbitrary, when not using the emulator, it will be the
        # current position of the motor when one starts the program
        self._position = 5.45
        self.is_in_motion = False
        self._stop = False

    @property
    def position(self):
        # In the GUI, I assumed that they give the position in mm
        return self._position

    @position.setter
    def position(self, value_mm):
        self.move_to(value_mm)

    def create_runnable(self, pos_mm):
        self.runnable = MotorRunnable(self, pos_mm)

    def move_to(self, value_mm):
        self.create_runnable(value_mm)
        pool.start(self.runnable)

    def move_by(self, value_mm, blocking=False):
        self.move_to(self._position + value_mm)

    def move_home(self, blocking):
        self.move_to(0.)

    def stop_profiled(self):
        self._stop = True

    def get_stage_axis_info(self):
        return 0, 10, "mm", 0.


class MotorRunnable(qtc.QRunnable):
    def __init__(self, motor, pos_mm):
        motor: Motor
        self.motor = motor
        self.pos_mm = pos_mm
        super().__init__()

    def run(self):
        # dx = sc.c * 1e-12 / 2
        dx = step_size_mm
        if self.motor.position < self.pos_mm:
            self.motor.is_in_motion = True

            while self.motor.position < self.pos_mm:
                if self.motor._stop:
                    self.motor._stop = False
                    self.motor.is_in_motion = False
                    return

                self.motor._position += dx
                time.sleep(sleep_time)

            self.motor.is_in_motion = False

        elif self.motor.position > self.pos_mm:
            self.motor.is_in_motion = True

            while self.motor.position > self.pos_mm:
                if self.motor._stop:
                    self.motor._stop = False
                    self.motor.is_in_motion = False
                    return

                self.motor._position -= dx
                time.sleep(sleep_time)

            self.motor.is_in_motion = False

        else:
            return


class Spectrometer:
    def __init__(self):
        self.int_time_micros = 1000
        self.integration_time_micros_limits = [1e3, 65e6]

    def wavelengths(self):
        return np.linspace(350, 1150, 5000)

    def spectrum(self):
        lambda0 = 25 + 5 * np.random.random()
        intensities = 1 / np.cosh((self.wavelengths() - 750) / lambda0)
        return self.wavelengths(), intensities

    def integration_time_micros(self, time_micros):
        self.int_time_micros = time_micros
