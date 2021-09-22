import struct
from APT import _auto_connect
import numpy as np
import APT as apt


class AptMotor(apt.KDC101_PRM1Z8):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.ENC_CNT_MM = 34304.

    @_auto_connect
    def position(self, position=None):
        if position is None:
            # Get the current position
            # MGMSG_MOT_REQ_POSCOUNTER 0x0411
            write_buffer = struct.pack("<BBBBBB", 0x11, 0x04,
                                       0x01, 0x00,
                                       self.dst, self.src)
            self.write(write_buffer)
            # MGMSG_MOT_GET_POSCOUNTER 0x0412
            read_buffer = self.read(0x12, 0x04, req_buffer=write_buffer)
            result = struct.unpack("<BBBBBBHl", read_buffer)
            position = result[7] / self.ENC_CNT_MM
            return position
        else:
            # Calculate the encoder value
            enc_cnt = int(round(position * self.ENC_CNT_MM))
            # MGMSG_MOT_MOVE_ABSOLUTE
            write_buffer = struct.pack('<BBBBBBHl', 0x53, 0x04,
                                       0x06, 0x00,
                                       self.dst | 0x80, self.src,
                                       0x0001, enc_cnt)
            self.write(write_buffer)

    @_auto_connect
    def move_relative(self, rel_position):
        enc_cnt = int(round(rel_position * self.ENC_CNT_MM))
        # MGMSG_MOT_MOVE_RELATIVE
        write_buffer = struct.pack("<BBBBBBHl", 0x48, 0x04,
                                   0x06, 0x00,
                                   self.dst | 0x80, self.src,
                                   0x0001, enc_cnt)
        self.write(write_buffer)

    @_auto_connect
    def stop(self):
        write_buffer = struct.pack("<6B", 0x65, 0x04, 0x01, 0x02,
                                   self.dst,
                                   self.src)
        self.write(write_buffer)


class KDC101(AptMotor):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    @property
    def is_in_motion(self):
        status = self.status()["flags"]
        to_check = [
            status["moving forward"],
            status["moving reverse"],
            status["jogging forward"],
            status["jogging reverse"],
            status["homing"]
        ]
        return np.any(to_check)

    @property
    def position(self):
        return super().position()

    @position.setter
    def position(self, value_mm):
        # assuming it's in millimeters
        super().position(position=value_mm)

    def move_to(self, value_mm):
        self.position = value_mm

    def move_by(self, value_mm, blocking=False):
        self.move_relative(value_mm)

    def move_home(self, *args):
        self.home(True)

    def stop_profiled(self):
        self.stop()

    def get_stage_axis_info(self):
        return 0., 25., "mm", None
