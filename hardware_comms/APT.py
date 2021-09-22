# -*- coding: utf-8 -*-
"""
Created on Tue Mar 19 10:06:34 2019

The communications protocol used in the Thorlabs controllers is based on the
message structure that always starts with a fixed length, 6-byte message header
which, in some cases, is followed by a variable length data packet. The header
part of the message always contains information that indicates whether or not a
data packet follows the header and if so, the number of bytes that the data
packet contains.

The 6 bytes in the message header are shown below:

                    Byte:   byte 0  byte 1  byte 2  byte 3  byte 4  byte 5
no data packet to follow    message ID      param1  param2  dest    source
data packet to follow       message ID      data length     dest    source

The meaning of some of the fields depends on whether or not the message is
followed by a data packet. This is indicated by the most significant bit in
byte 4, called the destination byte, therefore the receiving process must first
check if the MSB of byte 4 is set. If this bit is not set, then the message is
a header-only message and the interpretation of the bytes is as follows:
message ID: describes what the action the message requests
param1: first parameter (if the command requires a parameter, otherwise 0)
param2: second parameter (if the command requires a parameter, otherwise 0)
dest: the destination module
source: the source of the message

In all messages, where a parameter is longer than a single character, the bytes
are encoded in the Intel format, least significant byte first.

In non-card-slot type of systems the source and destination of messages is
always unambiguous, as each module appears as a separate USB node in the
system. In these systems, when the host sends a message to the module, it uses
the source identification byte of 0x01 (meaning host) and the destination byte
of 0x50 (meaning “generic USB unit”). In messages that the module sends back to
the host, the content of the source and destination bytes is swapped.

In card-slot (bay) type of systems, there is only one USB node for a number of
sub-modules, so this simple scheme cannot be used. Instead, the host sends a
message to the motherboard that the sub-modules are plugged into, with the
destination field of each message indicating which slot the message must be
routed to. Likewise, when the host receives a message from a particular
sub-module, it knows from the source byte which slot is the origin of the
message.

0x01    Host controller (i.e control PC)
0x11    Rack controller, motherboard in a card slot system or comms router board
0x21    Bay 0 in a card slot system
0x22    Bay 1 in a card slot system
0x23    etc.
0x24    etc.
0x25    etc.
0x26    etc.
...
0x2A    Bay 9 in a card slot system
0x50    Generic USB hardware unit

"""


# %% Modules

from functools import wraps
import struct
import serial
import time


# %% Private Functions

def _auto_connect(func):
    '''A function decorator that handles automatic connections.

    If "auto connect" is enabled the communications port is enabled before the
    function execution and disabled afterwards. If the internal "connected"
    flag is true, the connection/disconnection procedure is ignored and the
    function executes as normal.
    '''
    @wraps(func)
    def wrapper(self, *args, **kwargs):
        """Wrapped function"""
        if (self.auto_connect and not self.connected):
            try:
                self.open_port()
                result = func(self, *args, **kwargs)
                return result
            finally:
                self.close_port()
        else:
            result = func(self, *args, **kwargs)
            return result
    return wrapper


# %% APT Device

class APTDevice():
    def __init__(self, port, timeout=1., serial_number=None, source=0x01, destination=0x50):
        assert isinstance(port, str)

        self.auto_connect = True
        self.connected = False

        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = 115200
        self.ser.bytesize = serial.EIGHTBITS
        self.ser.parity = serial.PARITY_NONE
        self.ser.stopbits = serial.STOPBITS_ONE
        self.ser.timeout = timeout

        self.src = source # 0x01 = host controller
        self.dst = destination # 0x50 = generic usb device

        if serial_number is not None:
            assert serial_number == self.hardware_info()["serial"]
        
        self.send_update_messages(update=False)

    def open_port(self):
        '''Opens the serial port for read/write access'''
        if not self.ser.is_open:
            self.ser.open()
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.connected = True

    def close_port(self):
        '''Closes the serial port from read/write access'''
        if self.ser.is_open:
            self.ser.close()
            self.connected = False

    def read(self, msg_id_0, msg_id_1, req_buffer=None):
        '''Read and discard messages until the requested message ID is found.
        Returns the full buffer of the message or an error if the message is
        not found
        '''
        msg_found = False
        while not msg_found:
            read_buffer = self.ser.read(6)
            if len(read_buffer)==6:
                header = struct.unpack("<BBBBBB", read_buffer)
                # Check for data packet
                if bool(header[4] & 0x80):
                    packet_length = struct.unpack("<H", read_buffer[2:3+1])[0]
                    data_buffer = self.ser.read(packet_length)
                    read_buffer = read_buffer + data_buffer
                # Check msg_id
                #print("Message ID {:X},{:X} found.".format(header[0], header[1]))
                if msg_id_0 == header[0] and msg_id_1 == header[1]:
                    msg_found = True
                    return read_buffer
                else:
                    print('Message not found, trying again. {:X},{:X}'.format(header[0], header[1]))
            elif req_buffer is not None:
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                self.write(req_buffer)
                print('Message not found, trying again.')
                time.sleep(.1)
            else:
                raise ValueError("Message {:X},{:X} not found".format(msg_id_0, msg_id_1))
                

    def write(self, buffer):
        '''Writes buffer to serial port'''
        self.ser.write(buffer)

    @_auto_connect
    def identify(self):
        '''Causes the cube's LED screen to blink'''
        # MGMSG_MOD_IDENTIFY
        buffer = struct.pack("<BBBBBB", 0x23, 0x02,
                             0x00, 0x00,
                             self.dst, self.src)
        self.write(buffer)

    @_auto_connect
    def enable(self, enable=None, channel=1):
        '''Enable or disable the selected channel'''
        channel = {1:0x01, 2:0x02, 3:0x04, 4:0x08}[channel]
        if enable is None:
            # Check if the channel is enabled
            # MGMSG_MOD_REQ_CHANENABLESTATE
            write_buffer = struct.pack("<BBBBBB", 0x11, 0x02,
                                       channel, 0x00,
                                       self.dst, self.src)
            self.write(write_buffer)
            # MGMSG_MOD_GET_CHANENABLESTATE
            read_buffer = self.read(0x12, 0x02, req_buffer=write_buffer)
            result = struct.unpack("<BBBBBB", read_buffer)
            enable_state = {0x01:True, 0x02:False}[result[3]]
            return enable_state
        else:
            enable_state = {True:0x01, False:0x02}[bool(enable)]
            # MGMSG_MOD_SET_CHANENABLESTATE
            write_buffer = struct.pack("<BBBBBB", 0x10, 0x02,
                                       channel, enable_state,
                                       self.dst, self.src)
            self.write(write_buffer)

    @_auto_connect
    def hardware_info(self):
        '''Reports the hardware info of the connected cube'''
        # MGMSG_HW_REQ_INFO
        write_buffer = struct.pack("<BBBBBB", 0x05, 0x00,
                                   0x00, 0x00,
                                   self.dst, self.src)
        self.write(write_buffer)
        # MGMSG_HW_GET_INFO
        read_buffer = self.read(0x06, 0x00, req_buffer=write_buffer)
        result = struct.unpack("<BBBBBBL8sH4b60sHHH", read_buffer)
        return {"serial":       result[6],
                "model":        result[7].decode("ascii").strip("\x00"),
                "type":         result[8],
                "firmware":     '{:}.{:}.{:}.{:}'.format(*result[9:12+1]),
                "hardware":     result[14],
                "modification": result[15],
                "channels":     result[16]}

    @_auto_connect
    def send_update_messages(self, update=False, rate=0):
        '''Enable or disable a constant stream of status update messages'''
        if update == False:
            # MGMSG_HW_STOP_UPDATEMSGS
            write_buffer = struct.pack("<BBBBBB", 0x12, 0x00,
                                       0x00, 0x00,
                                       self.dst, self.src)
            self.write(write_buffer)
        else:
            # MGMSG_HW_START_UPDATEMSGS
            write_buffer = struct.pack("<BBBBBB", 0x11, 0x00,
                                       rate, 0x00,
                                       self.dst, self.src)
            self.write(write_buffer)


# %% KDC101 Brushed Motor Controler and PRM1Z8 Rotation Stage

class KDC101_PRM1Z8(APTDevice):
    # Conversion Factors
    ENC_CNT_DEG = 1919.64 # encoder counts per degree
    VEL_SCL_FCT = 42941.66 # encoder counts per (degrees per second)
    ACC_SCL_FCT = 14.66 # encoder counts per (degrees per second**2)

    def __init__(self, port, timeout=1, serial_number=None):
        super().__init__(port, timeout=timeout, serial_number=serial_number)

        # Suspend "End of Move Messages"
        self.suspend_EoM_msgs(True)

    @_auto_connect
    def suspend_EoM_msgs(self, suspend):
        '''Sent to disable or resume all unsolicited end of move messages and
        error messages returned by the controller:
            MGMSG_MOT_MOVE_STOPPED
            MGMSG_MOT_MOVE_COMPLETED
            MGMSG_MOT_MOVE_HOMED

        The command also disables the error messages that the controller sends
        when an error conditions is detected:
            MGMSG_HW_RESPONSE
            MGMSG_HW_RICHRESPONSE

        The messages are enabled by default when the controller is powered up.
        '''
        suspend = bool(suspend)
        if suspend:
            # MGMSG_MOT_SUSPEND_ENDOFMOVEMSGS
            write_buffer = struct.pack("<BBBBBB", 0x6B, 0x04,
                                       0x00, 0x00,
                                       self.dst, self.src)
            self.write(write_buffer)
        else:
            # MGMSG_MOT_RESUME_ENDOFMOVEMSGS
            write_buffer = struct.pack("<BBBBBB", 0x6C, 0x04,
                                       0x00, 0x00,
                                       self.dst, self.src)
            self.write(write_buffer)

    @_auto_connect
    def status(self):
        # MGMSG_MOT_REQ_DCSTATUSUPDATE
        write_buffer = struct.pack("<BBBBBB", 0x90, 0x04,
                                   0x01, 0x00,
                                   self.dst, self.src)
        self.write(write_buffer)
        # MGMSG_MOT_GET_DCSTATUSUPDATE
        read_buffer = self.read(0x91, 0x04, req_buffer=write_buffer)
        # MGMSG_MOT_ACK_DCSTATUSUPDATE
        ack_buffer = struct.pack("<BBBBBB", 0x92, 0x04,
                                 0x00, 0x00,
                                 self.dst, self.src)
        self.write(ack_buffer)
        # Unpack Read Buffer
        result = struct.unpack("<BBBBBBHlHHL", read_buffer)
        position = (result[7] / self.ENC_CNT_DEG) % 360 # degrees
        velocity = result[8] / self.VEL_SCL_FCT # degrees per second
        status_bits = {
            "forward hardware limit":   bool(result[10] & 0x00000001),
            "reverse hardware limit":   bool(result[10] & 0x00000002),
            "moving forward":           bool(result[10] & 0x00000010),
            "moving reverse":           bool(result[10] & 0x00000020),
            "jogging forward":          bool(result[10] & 0x00000040),
            "jogging reverse":          bool(result[10] & 0x00000080),
            "homing":                   bool(result[10] & 0x00000200),
            "homed":                    bool(result[10] & 0x00000400),
            "tracking":                 bool(result[10] & 0x00001000),
            "settled":                  bool(result[10] & 0x00002000),
            "motion error":             bool(result[10] & 0x00004000),
            "motor current limit":      bool(result[10] & 0x01000000),
            "channel enabled":          bool(result[10] & 0x80000000)}
        return {"position":position,
                "velocity":velocity,
                "flags":status_bits}

    @_auto_connect
    def home(self, home=None):
        if home is None:
            # Check if the device has been homed
            status_bits = self.status()["flags"]
            return {'homed':status_bits['homed'],
                    'homing':status_bits['homing']}
        elif home == True:
            # MGMSG_MOT_MOVE_HOME
            write_buffer = struct.pack("<BBBBBB", 0x43, 0x04,
                                       0x01, 0x00,
                                       self.dst, self.src)
            self.write(write_buffer)

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
            position = (result[7] / self.ENC_CNT_DEG) % 360 # degrees
            return position
        else:
            # Calculate the encoder value
            enc_cnt = int(round((position % 360) * self.ENC_CNT_DEG))
            # MGMSG_MOT_MOVE_ABSOLUTE
            write_buffer = struct.pack('<BBBBBBHl', 0x53, 0x04,
                                       0x06, 0x00,
                                       self.dst|0x80, self.src,
                                       0x0001, enc_cnt)
            self.write(write_buffer)

    @_auto_connect
    def move_relative(self, rel_position):
        enc_cnt = int(round(rel_position * self.ENC_CNT_DEG))
        # MGMSG_MOT_MOVE_RELATIVE
        write_buffer = struct.pack("<BBBBBBHl", 0x48, 0x04,
                                   0x06, 0x00,
                                   self.dst|0x80, self.src,
                                   0x0001, enc_cnt)
        self.write(write_buffer)


# %% KPZ101 K-Cube Piezo Controller

class KPZ101(APTDevice):
    # Position Control Mode
    OPEN_LOOP = 0x01
    CLOSED_LOOP = 0x02
    OPEN_LOOP_SMOOTH = 0x03
    CLOSED_LOOP_SMOOTH = 0x04

    # IO Settings
    VOLTAGELIMIT_75V = 0x01
    VOLTAGELIMIT_100V = 0x02
    VOLTAGELIMIT_150V = 0x03
    HUB_ANALOGUEIN_A = 0x01
    HUB_ANALOGUEIN_B = 0x02
    EXTSIG_SMA = 0x03

    # Output Voltage
    CNT_VLT_FR = 2**(16-1) - 1 # integer counts per max voltage
    MAX_VLT = [0, 75, 100, 150]

    # Input Voltage Source
    SOFTWARE_ONLY = 0x00
    EXTERNAL_SIGNAL = 0x01
    POTENTIOMETER = 0x02
    ALL_IN_SRCS = EXTERNAL_SIGNAL|POTENTIOMETER # 3

    def __init__(self, port, timeout=1, serial_number=None):
        super().__init__(port, timeout=timeout, serial_number=serial_number)

        # Set Position Control Mode
        self.position_control_mode(mode=self.OPEN_LOOP)

        # Populate IO Settings
        self.io_settings()

    @_auto_connect
    def position_control_mode(self, mode=None, persist=True):
        '''When in closed-loop mode, position is maintained by a feedback
        signal from the piezo actuator. This is only possible when using
        actuators equipped with position sensing.

        mode : int
            0x01    Open Loop (no feedback)

            0x02    Closed Loop (feedback employed)

            0x03    Open Loop Smooth

            0x04    Closed Loop Smooth

        If set to Open Loop Smooth or Closed Loop Smooth is selected, the
        feedback status is the same as above however the transition from open
        to closed loop (or vise versa) is achieved over a longer period in
        order to minimize voltage transients (spikes).
        '''
        if mode is None:
            # Get the current position control mode
            # MGMSG_PZ_REQ_POSCONTROLMODE
            write_buffer = struct.pack("<BBBBBB", 0x41, 0x06,
                                       0x01, 0x00,
                                       self.dst, self.src)
            self.write(write_buffer)
            # MGMSG_PZ_GET_POSCONTROLMODE
            read_buffer = self.read(0x42, 0x06, req_buffer=write_buffer)
            result = struct.unpack("<BBBBBB", read_buffer)
            mode = result[3]
            return mode
        else:
            assert mode in [self.OPEN_LOOP, self.CLOSED_LOOP, self.OPEN_LOOP_SMOOTH, self.CLOSED_LOOP_SMOOTH]
            # MGMSG_PZ_SET_POSCONTROLMODE
            write_buffer = struct.pack("<BBBBBB", 0x40, 0x06,
                                       0x01, mode,
                                       self.dst, self.src)
            self.write(write_buffer)
            if persist:
                # MGMSG_PZ_SET_EEPROMPARAMS 0x07D0
                write_buffer = struct.pack("<BBBBBBHBB", 0xD0, 0x07,
                                           0x04, 0x00,
                                           self.dst|0x80, self.src,
                                           0x0001, 0x40, 0x06)
                self.write(write_buffer)

    @_auto_connect
    def input_voltage_source(self, input_source=None, persist=True):
        '''Used to set the input source(s) which controls the output from the
        HV amplifier circuit (i.e. the drive to the piezo actuators).

        input_source : int
            0x00 = SOFTWARE_ONLY, Unit responds only to software inputs and the
            HV amp output is that set using the SetVoltOutput method or via the
            GUI panel.

            0x01 = EXTERNAL_SIGNAL, Unit sums the differential signal on the
            rear panel EXT IN (+) and EXT IN (-) connectors with the voltage
            set using the SetVoltOutput method

            0x02 = POTENTIOMETER, The HV amp output is controlled by a
            potentiometer input (either on the control panel, or connected to
            the rear panel User I/O D-type connector) summed with the voltage
            set using the SetVoltOutput method. => This is the "wheel" on the
            kcube.

            0x03 = ALL_IN_SRCS, The HV amp output is controlled by all of the
            above input sources.

        This function returns SOFTWARE_ONLY if the channel is disabled.
        '''
        if input_source is None:
            # MGMSG_PZ_REQ_INPUTVOLTSSRC 0x0653
            write_buffer = struct.pack("<BBBBBB", 0x53, 0x06,
                                       0x01, 0x00,
                                       self.dst, self.src)
            self.write(write_buffer)
            # MGMSG_PZ_GET_INPUTVOLTSSRC 0x0654
            read_buffer = self.read(0x54, 0x06, req_buffer=write_buffer)
            result = struct.unpack("<BBBBBBHH", read_buffer)
            voltage_source = result[7]
            return voltage_source
        else:
            assert input_source in [self.SOFTWARE_ONLY, self.EXTERNAL_SIGNAL, self.POTENTIOMETER, self.ALL_IN_SRCS]
            # MGMSG_PZ_SET_INPUTVOLTSSRC 0x0652
            write_buffer = struct.pack("<BBBBBBHH", 0x52, 0x06,
                                       0x04, 0x00,
                                       self.dst|0x80, self.src,
                                       0x0001, input_source)
            self.write(write_buffer)
            if persist:
                # MGMSG_PZ_SET_EEPROMPARAMS 0x07D0
                write_buffer = struct.pack("<BBBBBBHBB", 0xD0, 0x07,
                                           0x04, 0x00,
                                           self.dst|0x80, self.src,
                                           0x0001, 0x52, 0x06)
                self.write(write_buffer)

    @_auto_connect
    def io_settings(self, voltage_limit=None, analog_input=None, persist=True):
        '''This function is used to set various I/O settings.

        voltage_limit : int
            The piezo actuator connected to the T-Cube has a specific maximum
            operating voltage range. This parameter sets the maximum output to
            the value specified as follows...

            0x01 = VOLTAGELIMIT_75V,    75V limit

            0x02 = VOLTAGELIMIT_100V,   100V limit

            0x03 = VOLTAGELIMIT_150V,   150V limit

        analog_input : int
            When the K-Cube Piezo Driver unit is used a feedback signal can be
            passed from other cubes to the Piezo unit. This parameter is used
            to select the way in which the feedback signal is routed to the
            Piezo unit as follows...

            0x01 = HUB_ANALOGUEIN_A,    all cube bays

            0x02 = HUB_ANALOGUEIN_B,    adjacent pairs of cube bays (i.e. 1&2, 3&4, 5&6)

            0x03 = EXTSIG_SMA,          rear panel SMA connector
        '''
        if (voltage_limit is None) and (analog_input is None):
            # Get the current IO settings
            # MGMSG_PZ_REQ_TPZ_IOSETTINGS
            write_buffer = struct.pack("<BBBBBB", 0xD5, 0x07,
                                       0x01, 0x00,
                                       self.dst, self.src)
            self.write(write_buffer)
            # MGMSG_PZ_GET_TPZ_IOSETTINGS
            read_buffer = self.read(0xD6, 0x07, req_buffer=write_buffer)
            result = struct.unpack("<BBBBBBHHHHH", read_buffer)
            voltage_limit = result[7]
            analog_input = result[8]
            self._voltage_limit = voltage_limit
            self._analog_input = analog_input
            return {"voltage_limit":voltage_limit,
                    "analog_input":analog_input}
        else:
            if voltage_limit is None:
                voltage_limit = self._voltage_limit
            if analog_input is None:
                analog_input = self._analog_input
            # Check values
            assert voltage_limit in [self.VOLTAGELIMIT_75V, self.VOLTAGELIMIT_100V, self.VOLTAGELIMIT_150V]
            assert analog_input in [self.HUB_ANALOGUEIN_A, self.HUB_ANALOGUEIN_B, self.EXTSIG_SMA]
            # MGMSG_PZ_SET_TPZ_IOSETTINGS
            write_buffer = struct.pack("<BBBBBBHHHHH", 0xD4, 0x07,
                                       0x0A, 0x00,
                                       self.dst|0x80, self.src,
                                       0x0001, voltage_limit, analog_input, 0, 0)
            self.write(write_buffer)
            self._voltage_limit = voltage_limit
            self._analog_input = analog_input
            if persist:
                # MGMSG_PZ_SET_EEPROMPARAMS 0x07D0
                write_buffer = struct.pack("<BBBBBBHBB", 0xD0, 0x07,
                                           0x04, 0x00,
                                           self.dst|0x80, self.src,
                                           0x0001, 0xD4, 0x07)
                self.write(write_buffer)

    @_auto_connect
    def voltage(self, voltage=None):
        '''Used to set the output voltage applied to the piezo actuator.

        This command is applicable only in Open Loop mode. If called when in
        Closed Loop mode it is ignored.

        voltage : float
            The output voltage applied to the piezo when operating in open loop
            mode. The voltage is scaled into the range -32768 to 32767 (-0x7FFF
            to 0x7FFF) to which corresponds to -100% to 100% of the maximum
            output voltage as set using the TPZ_IOSETTINGS command.
        '''
        if voltage is None:
            # Get the current voltage output
            # MGMSG_PZ_REQ_OUTPUTVOLTS
            write_buffer = struct.pack("<BBBBBB", 0x44, 0x06,
                                       0x01, 0x00,
                                       self.dst, self.src)
            self.write(write_buffer)
            # MGMSG_PZ_GET_OUTPUTVOLTS
            read_buffer = self.read(0x45, 0x06, req_buffer=write_buffer)
            result = struct.unpack("<BBBBBBHh", read_buffer)
            voltage = result[7]/self.CNT_VLT_FR * self.MAX_VLT[self._voltage_limit]
            return voltage
        else:
            assert (voltage >= 0) and (voltage <= self.MAX_VLT[self._voltage_limit])
            # MGMSG_PZ_SET_OUTPUTVOLTS
            voltage_cnt = int(round(voltage/self.MAX_VLT[self._voltage_limit] * self.CNT_VLT_FR))
            write_buffer = struct.pack("<BBBBBBHh", 0x43, 0x06,
                                       0x04, 0x00,
                                       self.dst|0x80, self.src,
                                       0x0001, voltage_cnt)
            self.write(write_buffer)


# %% TNA001 T-Cube NanoTrack Alignment Controller

class TNA001(APTDevice):

    # NanoTrack Mode
    LATCH_MODE = 0x02
    TRACK_MODE = 0x03
    H_TRACK_MODE = 0x04
    V_TRACK_MODE = 0x05

    # NanoTrack Status
    TRCK_NO_SGN = 0x03
    TRCK_NORM = 0x04
    DUAL_AXIS_TRK = 0x01
    HORZ_TRK = 0x02
    VERT_TRK = 0x03

    # Position
    CNT_FR_POS = 2**(16)-1 # counts per full range output position

    # Sampling Frequency
    SMP_FRQ = 7000 # Hz

    # Circle Parameters
    NTCIRCDIA_SW = 0x01
    NTCIRCDIA_ABSPWR = 0x02
    NTCIRCDIA_LUT = 0x03
    NTABSPWRCIRCADJUST_LIN = 0x01
    NTABSPWRCIRCADJUST_LOG = 0x02
    NTABSPWRCIRCADJUST_X2 = 0x03
    NTABSPWRCIRCADJUST_X3 = 0x04

    # Phase Compensation
    SFTW_PHS_ADJ = 0x0002

    # TIA Range Parameters
    RANGE_AUTO = 0x01
    RANGE_SW = 0x02
    RANGE_SWSET = 0x03
    RANGE_AUTOSET = 0x04
    CNT_FR_TIA = 1000 # counts per full range TIA range
    AUTORANGE_ALL = 0x01
    AUTORANGE_ODD = 0x02
    AUTORANGE_EVEN = 0x03
    RANGE_1 = 0x03
    RANGE_2 = 0x04
    RANGE_3 = 0x05
    RANGE_4 = 0x06
    RANGE_5 = 0x07
    RANGE_6 = 0x08
    RANGE_7 = 0x09
    RANGE_8 = 0x0A
    RANGE_9 = 0x0B
    RANGE_10 = 0x0C
    RANGE_11 = 0x0D
    RANGE_12 = 0x0E
    RANGE_13 = 0x0F
    RANGE_14 = 0x10

    # Gain
    GAIN_SW = 0x02

    # TIA Reading
    NORM_READ = 0x01
    UNDER_READ = 0x02
    OVER_READ = 0x03

    # Feedback Source
    P_PZ_NTFBTIA = 0x01
    P_PZ_NTFBBNC1V = 0x02
    P_PZ_NTFBBNC2V = 0x03
    P_PZ_NTFBBNC5V = 0x04
    P_PZ_NTFBBNC10V = 0x05

    # IO Settings
    RANGE_5V = 0x01
    RANGE_10V = 0x02
    RT_SMA = 0x01
    RT_SMA_HUB = 0x02

    def __init__(self, port, timeout=1, serial_number=None):
        super().__init__(port, timeout=timeout, serial_number=serial_number)
        self.enable = lambda enable=None, channel=1: True

        # Get Current Position
        self.position()

        # Get Circle Parameters
        self.circle_parameters()

        # Phase Compensation
        self.phase_comp()

        # TIA Parameters
        self.tia_range_parameters()

        # IO Settings
        self.io_settings()

    @_auto_connect
    def track_mode(self, mode=None):
        '''This message gets the present operating mode of the unit.

        mode : int
            0x01 = PIEZO_MODE, NanoTracking off. The unit is in Piezo mode

            0x02 = LATCH_MODE, In this mode, scanning is disabled and the piezo
            drives are held at the present position.

            0x03 = TRACK_MODE, In this mode, the NanoTrak detects any drop in
            signal strength resulting from misalignment of the input and output
            devices, and makes vertical and horizontal positional adjustments
            to maintain the maximum throughput.

            0x04 = H_TRACK_MODE, In this mode, the NanoTrak detects any
            drop in signal strength resulting from misalignment of the input
            and output devices, and makes horizontal positional adjustments to
            maintain the maximum throughput.

            0x05 = V_TRACK_MODE, In this mode, the NanoTrak detects any drop
            in signal strength resulting from misalignment of the input and
            output devices, and makes vertical positional adjustments to
            maintain the maximum throughput.
        '''
        if mode is None:
            # MGMSG_PZ_REQ_NTMODE 0x0604
            write_buffer = struct.pack("<BBBBBB", 0x04, 0x06,
                                       0x00, 0x00,
                                       self.dst, self.src)
            self.write(write_buffer)
            # MGMSG_PZ_GET_NTMODE 0x0605
            read_buffer = self.read(0x05, 0x06, req_buffer=write_buffer)
            result = struct.unpack("<BBBBBB", read_buffer)
            mode = result[2]
            if (mode == self.TRCK_NO_SGN) or (mode == self.TRCK_NORM):
                track_type = result[3]
                mode = {self.DUAL_AXIS_TRK:self.TRACK_MODE,
                        self.HORZ_TRK:self.H_TRACK_MODE,
                        self.VERT_TRK:self.V_TRACK_MODE}[track_type]
            return mode
        else:
            assert mode in [self.LATCH_MODE, self.TRACK_MODE, self.H_TRACK_MODE, self.V_TRACK_MODE]
            # MGMSG_PZ_SET_NTMODE 0x0603
            write_buffer = struct.pack("<BBBBBB", 0x03, 0x06,
                                       mode, 0x00,
                                       self.dst, self.src)
            self.write(write_buffer)

    @_auto_connect
    def track_threshold(self, threshold=None):
        '''This message sets the tracking threshold of the NanoTrak.

        The value is set in Amps, and is dependent upon the application.
        Typically, the value is set to lie above the ‘noise floor’ of the
        particular physical arrangement. When the input signal level exceeds
        this value, the tracking LED is lit on the GUI panel. Note there is no
        guarantee that tracking is taking place if this threshold value is set
        inappropriately.

        threshold
            The tracking threshold of the NanoTrak. This is the absolute TIA
            reading (PIN current). The value set in Amps as a 4-byte floating
            point number in the range 1e-9 to 1e-3 (i.e. 1 nA to 1 mA).
        '''
        if threshold is None:
            # MGMSG_PZ_REQ_NTTRACKTHRESHOLD 0x0607
            write_buffer = struct.pack("<BBBBBB", 0x07, 0x06,
                                       0x00, 0x00,
                                       0x50, 0x01)
            self.write(write_buffer)
            # MGMSG_PZ_GET_NTTRACKTHRESHOLD 0x0608
            read_buffer = self.read(0x08, 0x06, req_buffer=write_buffer)
            result = struct.unpack("<BBBBBBf", read_buffer)
            threshold = result[6]
            return threshold
        else:
            assert (threshold <= 1e-3) and (threshold >= 1e-9)
            # MGMSG_PZ_SET_NTTRACKTHRESHOLD 0x0606
            write_buffer = struct.pack("<BBBBBBf", 0x06, 0x06,
                                       0x04, 0x00,
                                       self.dst|0x80, self.src,
                                       threshold)
            self.write(write_buffer)

    @_auto_connect
    def position(self, x=None, y=None):
        '''The horizontal and vertical position of the circle.

        x : float
            The horizontal co-ordinate of the circle home position, in the
            range 0 to 1 (0 to 100% of output voltage or 0 to 65535 NanoTrak
            units).

        y : float
            The vertical co-ordinate of the circle home position, in the range
            0 to 1 (0 to 100% of output voltage or 0 to 65535 NanoTrak units).
        '''
        if (x is None) and (y is None):
            # MGMSG_PZ_REQ_NTCIRCCENTREPOS 0x0613
            write_buffer = struct.pack("<BBBBBB", 0x13, 0x06,
                                       0x01, 0x00,
                                       self.dst, self.src)
            self.write(write_buffer)
            # MGMSG_PZ_GET_NTCIRCCENTREPOS 0x0614
            read_buffer = self.read(0x14, 0x06, req_buffer=write_buffer)
            result = struct.unpack("<BBBBBBHHfHHH", read_buffer)
            x = result[6] / self.CNT_FR_POS
            y = result[7] / self.CNT_FR_POS
            self._x_pos = x
            self._y_pos = y
            return {"x":x, "y":y}
        else:
            if y is None:
                y = self._y_pos
            if x is None:
                x = self._x_pos
            assert (x >= 0) and (x <= 1) and (y >= 0) and (y <= 1)
            # MGMSG_PZ_SET_NTCIRCHOMEPOS 0x0609
            x_cnts = int(round(x * self.CNT_FR_POS))
            y_cnts = int(round(y * self.CNT_FR_POS))
            write_buffer = struct.pack("<BBBBBBHH", 0x09, 0x06,
                                       0x04, 0x00,
                                       self.dst|0x80, self.src,
                                       x_cnts, y_cnts)
            self.write(write_buffer)
            # MGMSG_PZ_MOVE_NTCIRCTOHOMEPOS 0x0612
            write_buffer = struct.pack("<BBBBBB", 0x12, 0x06,
                                       0x00, 0x00,
                                       self.dst, self.src)
            self.write(write_buffer)
            self._x_pos = x_cnts / self.CNT_FR_POS
            self._y_pos = y_cnts / self.CNT_FR_POS

    @_auto_connect
    def circle_parameters(self, params=None, persist=True):
        '''
        mode : int
            This parameter allows the different modes of circle diameter
            adjustment to be enabled and disabled as follows:

            0x01 = NTCIRCDIA_SW, the circle diameter remains at the value set
            using the CircDiaSW parameter below.

            0x02 = NTCIRCDIA_ABSPWR, the circle diameter is set by absolute
            power input value (depending on adjustment algorithm selected in
            the AbsPwrAdjustType parameter - see below). Not applicable on the
            TNA001.

            0x03 = NTCIRCDIA_LUT, the circle diameter is adjusted
            automatically, using a table of TIA range dependent values (set
            using the SetCircDiaLUT message.

        diameter : float
            This parameter sets the NT circle diameter if NTCIRCDIA_SW (0x01)
            is selected in the CircDiaMode parameter above. The diameter is set
            in the range 0 to 1, which relates to 0% to 100% output voltage
            (i.e. 0 to 65535 NT units).

        frequency : float
            The scanning frequency is used to set the number of samples taken
            in one revolution of the scanning circle. The circle scanning
            frequency lies in the range 17.5 Hz to 87.5 Hz for TNA001. The
            factory default setting for the scanning frequency is 43.75Hz. This
            means that a stage driven by the NanoTrak makes 43.75 circular
            movements per second. Different frequency settings allow more than
            one NanoTrak to be used in the same alignment scenario. The
            scanning frequency is derived from the NanoTrak sampling frequency
            of 7000 Hz and the CircOscFreq value which is calculated as
            follows:

            CircOscFreq = 7000 / frequency

            Note: The CircOscFreq parameter must be entered as a multiple of
            "4"

        min diameter : float
            The minimum circle diameter. Applicable only if the CircDiaMode
            parameter above is set to NTCIRCDIA_ABSPWR (0x02). The diameter is
            set in the range 0 to 0.5, which relates to 0% to 50% output
            voltage –(i.e. 0 to 32767 NT units).

        max diameter : float
            The maximum circle diameter. Applicable only if the CircDiaMode
            parameter above is set to NTCIRCDIA_ABSPWR (0x02). The diameter is
            set in the range 0 to 0.5, which relates to 0% to 50% output
            voltage –(i.e. 0 to 32767 NT units).

        adjust type : int
            This parameter sets the adjustment type and is applicable only if
            mode parameter above is set to NTCIRCDIA_ABSPWR (0x02).

            0x01 = NTABSPWRCIRCADJUST_LIN, inverse linear adjustment

            0x02 = NTABSPWRCIRCADJUST_LOG, inverse log adjustment

            0x03 = NTABSPWRCIRCADJUST_X2, inverse square adjustment

            0x04 = NTABSPWRCIRCADJUST_X3, inverse cube adjustment
        '''
        if params is None:
            # MGMSG_PZ_REQ_NTCIRCPARAMS 0x0619
            write_buffer = struct.pack("<BBBBBB", 0x19, 0x06,
                                       0x01, 0x00,
                                       self.dst, self.src)
            self.write(write_buffer)
            # MGMSG_PZ_GET_NTCIRCPARAMS 0x0620
            read_buffer = self.read(0x20, 0x06, req_buffer=write_buffer)
            result = struct.unpack("<BBBBBBHHHHHH", read_buffer)
            params = {
                "mode":         result[6],
                "diameter":     result[7]/self.CNT_FR_POS,
                "frequency":    self.SMP_FRQ/result[8],
                "min diameter": result[9]/self.CNT_FR_POS,
                "max diameter": result[10]/self.CNT_FR_POS,
                "adjust type":  result[11]}
            self._circ_params = params
            return params
        else:
            assert isinstance(params, dict)
            if "mode" not in params:
                params["mode"] = self._circ_params["mode"]
            if "diameter" not in params:
                params["diameter"] = self._circ_params["diameter"]
            if "frequency" not in params:
                params["frequency"] = self._circ_params["frequency"]
            if "min diameter" not in params:
                params["min diameter"] = self._circ_params["min diameter"]
            if "max diameter" not in params:
                params["max diameter"] = self._circ_params["max diameter"]
            if "adjust type" not in params:
                params["adjust type"] = self._circ_params["adjust type"]
            # Circle Diameter Mode
            assert params["mode"] in [self.NTCIRCDIA_SW, self.NTCIRCDIA_LUT]
            circ_dia_mode = params["mode"]
            # Circle Diameter Set Point
            assert (params["diameter"] >= 0) and (params["diameter"] <= 1)
            circ_dia_sw = int(round(params["diameter"] * self.CNT_FR_POS))
            # Circle Oscillation Frequency
            assert (params["frequency"] >= 17.5) and (params["frequency"] <= 87.5)
            circ_osc_freq = int(round(self.SMP_FRQ / params["frequency"]))
            circ_osc_freq = circ_osc_freq - (circ_osc_freq % 4) # must be a multiple of 4
            # Absolute Power Min Circle Diameter
            assert (params["min diameter"] >= 0) and (params["min diameter"] <= 0.5)
            abs_pwr_min_circ_dia = int(round(params["min diameter"] * self.CNT_FR_POS))
            # Absolute Power Max Circle Diameter
            assert (params["max diameter"] >= 0) and (params["max diameter"] <= 0.5)
            abs_pwr_max_circ_dia = int(round(params["max diameter"] * self.CNT_FR_POS))
            # Absolute Power Adjustment Type
            assert params["adjust type"] in [self.NTABSPWRCIRCADJUST_LIN, self.NTABSPWRCIRCADJUST_LOG, self.NTABSPWRCIRCADJUST_X2, self.NTABSPWRCIRCADJUST_X3]
            abs_pwr_adjust_type = params["adjust type"]
            # MGMSG_PZ_SET_NTCIRCPARAMS 0x0618
            write_buffer = struct.pack("<BBBBBBHHHHHH", 0x18, 0x06,
                                       0x0C, 0x00,
                                       self.dst|0x80, self.src,
                                       circ_dia_mode, circ_dia_sw,
                                       circ_osc_freq,
                                       abs_pwr_min_circ_dia,
                                       abs_pwr_max_circ_dia,
                                       abs_pwr_adjust_type)
            self.write(write_buffer)
            self._circ_params = {
                "mode":         circ_dia_mode,
                "diameter":     circ_dia_sw/self.CNT_FR_POS,
                "frequency":    self.SMP_FRQ/circ_osc_freq,
                "min diameter": abs_pwr_min_circ_dia/self.CNT_FR_POS,
                "max diameter": abs_pwr_max_circ_dia/self.CNT_FR_POS,
                "adjust type":  abs_pwr_adjust_type}
            if persist:
                # MGMSG_NT_SET_EEPROMPARAMS 0x07E7
                write_buffer = struct.pack("<BBBBBBHBB", 0xE7, 0x07,
                                           0x04, 0x00,
                                           self.dst|0x80, self.src,
                                           0x0001, 0x18, 0x06)
                self.write(write_buffer)

    @_auto_connect
    def phase_comp(self, x=None, y=None, persist=True):
        '''The closed loop phase compensation factors.

        The feedback loop scenario in a typical NanoTrak application can
        involve the operation of various electronic and electromechanical
        components (e.g. power meters and piezo actuators) that could
        introduce phase shifts around the loop and thereby affect tracking
        efficiency and stability. These phase shifts can be cancelled by
        setting the 'Phase Compensation' factors. This message sets the
        phase compensation for the horizontal and vertical components of
        the circle path in the range 0 to 360 degrees. Typically both phase
        offsets will be set the same, although some electromechanical
        systems may exhibit different phase lags in the different
        components of travel and so require different values.

        An optimization algorithm could be to minimize the response of the
        tracker while in H_TRACK_MODE or V_TRACK_MODE, and then add +-90
        degrees to bring the loop in phase.

        PhaseCompASW
            The horizontal axis phase compensation value, entered in real
            world units and calculated as follows:

            value = (phase angle [degrees] / 360) * CircOscFreq

        PhaseCompBSW
            The vertical axis phase compensation value, entered in real
            world units and calculated as follows:

            value = (phase angle [degrees] / 360) * CircOscFreq

        Notes
        -----
        See the PZ_SET_NTCIRCPARAMS message for details on the CircOscFreq
        parameter.

        Negative phase values must be made positive by subtraction from 360
        before the calculation is made.

        Currently, the phase compensation mode is not adjustable, and is
        locked at manual (software) adjustment.

        0x0002 = SFTW_PHS_ADJ, software adjustment mode
        '''
        if (x is None) and (y is None):
            # MGMSG_PZ_REQ_NTPHASECOMPPARAMS 0x0627
            write_buffer = struct.pack("<BBBBBB", 0x27, 0x06,
                                       0x00, 0x00,
                                       self.dst, self.src)
            self.write(write_buffer)
            # MGMSG_PZ_GET_NTPHASECOMPPARAMS 0x0628
            read_buffer = self.read(0x28, 0x06, req_buffer=write_buffer)
            result = struct.unpack("<BBBBBBHhh", read_buffer)
            x = result[7]/(self.SMP_FRQ / self._circ_params["frequency"]) * 360
            y = result[8]/(self.SMP_FRQ / self._circ_params["frequency"]) * 360
            self._x_phase = x
            self._y_phase = y
            return {"x": x, "y": y}
        else:
            if x is None:
                x = self._x_phase
            if y is None:
                y = self._y_phase
            phase_comp_a_sw = int(round((x % 360)/360 * (self.SMP_FRQ / self._circ_params["frequency"])))
            phase_comp_b_sw = int(round((y % 360)/360 * (self.SMP_FRQ / self._circ_params["frequency"])))
            # MGMSG_PZ_SET_NTPHASECOMPPARAMS 0x0626
            write_buffer = struct.pack("<BBBBBBHhh", 0x26, 0x06,
                                       0x06, 0x00,
                                       self.dst|0x80, self.src,
                                       self.SFTW_PHS_ADJ,
                                       phase_comp_a_sw,
                                       phase_comp_b_sw)
            self.write(write_buffer)
            self._x_phase = phase_comp_a_sw/(self.SMP_FRQ / self._circ_params["frequency"]) * 360
            self._y_phase = phase_comp_b_sw/(self.SMP_FRQ / self._circ_params["frequency"]) * 360
            if persist:
                # MGMSG_NT_SET_EEPROMPARAMS 0x07E7
                write_buffer = struct.pack("<BBBBBBHBB", 0xE7, 0x07,
                                           0x04, 0x00,
                                           self.dst|0x80, self.src,
                                           0x0001, 0x26, 0x06)
                self.write(write_buffer)

    @_auto_connect
    def tia_range_parameters(self, params=None):
        '''
        mode : int
            This parameter specifies the ranging mode of the unit as follows:

            0x01 = RANGE_AUTO, change to Auto ranging at the range currently
            selected

            0x02 = RANGE_SW, change to manual ranging at the range currently
            selected

            0x03 = RANGE_SWSET, change to manual ranging at the range set in
            the SetRange method (or the 'Settings' panel)

            0x04 = RANGE_AUTOSET, change to Auto ranging at the range set in
            the RangeSW parameter below.

        range up : float
            Only applicable if Auto Ranging is selected in the RangeMode
            parameter above. This parameter sets the upper range limit as a
            percentage of the present range, 0 to 1000 = 0 to 100%. When
            autoranging, the NanoTrak unit adjusts continually the TIA range as
            appropriate for the input signal level. When the relative signal
            rises above the limit specified in this parameter, the unit
            increments the range to the next higher setting. The relative
            signal is displayed on the NanoTrak GUI panel by a green
            horizontal bar.

        range down : float
            Only applicable if Auto Ranging is selected in the RangeMode
            parameter above. This parameter sets the lower range limit as a
            percentage of the present range, 0 to 1000 = 0 to 100%. Similarly
            to RangeUpLimit, when the relative signal on a particular range
            drifts below the limit set in this parameter, the NanoTrak unit
            decrements the range to the next lower setting. The relative signal
            is displayed on the NanoTrak GUI panel by a green horizontal bar.

        samples : int
            Only applicable if Auto Ranging is selected in the RangeMode
            parameter above. This parameter determines the amount of averaging
            applied to the signal before autoranging takes place. Higher
            SettleSamples values improve the signal to noise ratio when dealing
            with noisy feedback signals. However, higher SettleSamples values
            also slow down the autoranging response. In a particular
            application, the SettleSamples value should be adjusted to obtain
            the best autoranging response combined with a noise free signal.
            Values are set in real world units, from ‘2’ to ‘32’, with a
            default setting value of ‘4’.

        auto type : int
            Only applicable if Auto Ranging is selected in the RangeMode
            parameter above. This parameter specifies how range changes are
            implemented by the system.

            0x01 = AUTORANGE_ALL, the unit visits all ranges when ranging
            between two input signal levels.

            0x02 = AUTORANGE_ODD, only the odd numbered ranges between the two
            input signals levels will be visited.

            0x03 = AUTORANGE_EVEN, only the even numbered ranges between the
            two input signals levels will be visited.

            These latter two modes are useful when large rapid input signal
            fluctuations are anticipated, because the number of ranges visited
            is halved to give a more rapid response.

        range : int
            Only applicable if Manual (SW) Ranging is selected in the RangeMode
            parameter above. The NanoTrak unit is equipped with an internal
            trans-impedance amplifier (TIA) circuit (and associated range/power
            level displays and control buttons in the GUI). This amplifier
            operates when an external input signal is connected to the
            Optical/PIN connector on the rear panel. There are 14 range
            settings (1 - 14) that can be used to select the best range to
            measure the input signal (displayed on the GUI panel relative input
            signal bar and display).

            Note. Range 1 and 2 (3 nA and 10 nA) are not applicable to TNA001
            T-Cube units.

            0x03 = RANGE_1, 3 nA

            0x04 = RANGE_2, 10 nA

            0x05 = RANGE_3, 30 nA

            0x06 = RANGE_4, 100 nA

            0x07 = RANGE_5, 300 nA

            0x08 = RANGE_6, 1 μA

            0x09 = RANGE_7, 3 μA

            0x0A = RANGE_8, 10 μA

            0x0B = RANGE_9, 30 μA

            0x0C = RANGE_10, 100 μA

            0x0D = RANGE_11, 300 μA

            0x0E = RANGE_12, 1 mA

            0x0F = RANGE_13, 3 mA

            0x10 = RANGE_14, 10 mA
        '''
        if params is None:
            # MGMSG_PZ_REQ_NTTIARANGEPARAMS 0x0631
            write_buffer = struct.pack("<BBBBBB", 0x31, 0x06,
                                       0x01, 0x00,
                                       self.dst, self.src)
            self.write(write_buffer)
            # MGMSG_PZ_GET_NTTIARANGEPARAMS 0x0632
            read_buffer = self.read(0x32, 0x06, req_buffer=write_buffer)
            result = struct.unpack("<BBBBBBHhhhHH", read_buffer)
            params = {
                "mode":         result[6],
                "range up":     result[7] / self.CNT_FR_TIA,
                "range down":   result[8] / self.CNT_FR_TIA,
                "samples":      result[9],
                "auto type":    result[10],
                "range":        result[11]}
            self._tia_params = params
            return params
        else:
            if "mode" not in params:
                params["mode"] = self._tia_params["mode"]
            if "range up" not in params:
                params["range up"] = self._tia_params["range up"]
            if "range down" not in params:
                params["range down"] = self._tia_params["range down"]
            if "samples" not in params:
                params["samples"] = self._tia_params["samples"]
            if "auto type" not in params:
                params["auto type"] = self._tia_params["auto type"]
            if "range" not in params:
                params["range"] = self._tia_params["range"]
            # Range Mode
            assert params["mode"] in [self.RANGE_AUTO, self.RANGE_SW, self.RANGE_SWSET, self.RANGE_AUTOSET]
            range_mode = params["mode"]
            # Range Up Limit
            assert (params["range up"] >= 0) and (params["range up"] <= 1)
            range_up_limit = int(round(params["range up"] * self.CNT_FR_TIA))
            # Range Down Limit
            assert (params["range down"] >= 0) and (params["range down"] <= 1)
            range_down_limit = int(round(params["range down"] * self.CNT_FR_TIA))
            # Settle Samples
            assert (params["samples"] >= 2) and (params["samples"] <= 32)
            settle_samples = int(params["samples"])
            # Range Change Type
            assert params["auto type"] in [self.AUTORANGE_ALL, self.AUTORANGE_ODD, self.AUTORANGE_EVEN]
            range_change_type = params["auto type"]
            # Manual Range
            assert params["range"] in [self.RANGE_3, self.RANGE_4, self.RANGE_5,
                                       self.RANGE_6, self.RANGE_7, self.RANGE_8, self.RANGE_9,
                                       self.RANGE_10, self.RANGE_11, self.RANGE_12, self.RANGE_13, self.RANGE_14]
            range_sw = params["range"]
            # MGMSG_PZ_SET_NTTIARANGEPARAMS 0x0630
            write_buffer = struct.pack("<BBBBBBHhhhHH", 0x30, 0x06,
                                       0x0C, 0x00,
                                       self.dst|0x80, self.src,
                                       range_mode,
                                       range_up_limit, range_down_limit,
                                       settle_samples,
                                       range_change_type,
                                       range_sw)
            self.write(write_buffer)
            self._tia_params = {
                "mode":         range_mode,
                "range up":     range_up_limit / self.CNT_FR_TIA,
                "range down":   range_down_limit / self.CNT_FR_TIA,
                "samples":      settle_samples,
                "auto type":    range_change_type,
                "range":        range_sw}

    @_auto_connect
    def gain(self, gain=None, persist=True):
        '''
        gain : int
            This parameter sets the loop gain, as a function of TIA range
            setting. The value is set between 100 and 10000 with a default
            value of 600. It is not normally necessary for anything other than
            minor adjustment from this default value.

        Note
        ----
        GainCtrlMode
            This parameter is currently locked and cannot be changed:

            0x02 = GAIN_SW, software setting gain control mode
        '''
        if gain is None:
            # MGMSG_PZ_REQ_NTGAINPARAMS 0x0634
            write_buffer = struct.pack("<BBBBBB", 0x34, 0x06,
                                       0x00, 0x00,
                                       self.dst, self.src)
            self.write(write_buffer)
            # MGMSG_PZ_GET_NTGAINPARAMS 0x0635
            read_buffer = self.read(0x35, 0x06, req_buffer=write_buffer)
            result = struct.unpack("<BBBBBBHh", read_buffer)
            gain = result[7]
            return gain
        else:
            assert (gain >= 100) and (gain <= 10000)
            gain_sw = int(gain)
            # MGMSG_PZ_SET_NTGAINPARAMS 0x0633
            write_buffer = struct.pack("<BBBBBBHh", 0x33, 0x06,
                                       0x04, 0x00,
                                       self.dst|0x80, self.src,
                                       self.GAIN_SW,
                                       gain_sw)
            self.write(write_buffer)
            if persist:
                # MGMSG_NT_SET_EEPROMPARAMS 0x07E7
                write_buffer = struct.pack("<BBBBBBHBB", 0xE7, 0x07,
                                           0x04, 0x00,
                                           self.dst|0x80, self.src,
                                           0x0001, 0x33, 0x06)
                self.write(write_buffer)

    @_auto_connect
    def tia_reading(self):
        '''
        AbsReading
            This parameter returns the absolute TIA (PIN) current or BNC
            voltage value at the current position. The value is returned as a
            4 byte floating point value in the range 1e-9 to 1e-3 (i.e. 1 nA
            to 1 mA or 1 to 10 V). The input source, TIA or BNC is set in the
            Set_NTFeedbackSRC message.

        RelReading
            The relative signal strength at the current position, in the range
            0 to 1 (i.e. 0 to 100% of the range currently selected). This
            value matches the length of the input signal bargraph on the GUI
            panel. (e.g. if the 3 μA range is currently selected, then a
            RelReading value of 0.5 (50%) equates to 1.5 μA).).

        Range
            This parameter returns the input signal range currently selected.
            There are 14 range settings (1 - 14) that can be used to select the
            best range to measure the input signal (displayed on the GUI panel
            relative input signal bar and display).

        UnderOverRead
            This parameter returns a value that identifies whether the unit is
            under reading or over reading the input signal as follows:

            0x01 = NORM_READ, power signal is within current TIA range

            0x02 = UNDER_READ, power signal is under-reading for current TIA

            0x03 = OVER_READ, power signal is over-reading for current TIA
            range

            e.g. if a user specified range of 3 μA is currently applied, this
            parameter returns '0x03’ (Over read)' for input signals greater
            than 3 μA.
        '''
        # MGMSG_PZ_REQ_NTTIAREADING 0x0639
        write_buffer = struct.pack("<BBBBBB", 0x39, 0x06,
                                   0x00, 0x00,
                                   self.dst, self.src)
        self.write(write_buffer)
        # MGMSG_PZ_GET_NTTIAREADING 0x063A
        read_buffer = self.read(0x3A, 0x06, req_buffer=write_buffer)
        result = struct.unpack("<BBBBBBfHHH", read_buffer)
        return {"abs reading":result[6],
                "rel reading":result[7]/(self.CNT_FR_POS//2),
                "range":result[8],
                "under over":result[9]}

    @_auto_connect
    def feedback_source(self, source=None, persist=True):
        '''This message sets the input source of the NanoTrak.

        The INPUT_BNC settings are used when NanoTraking to optimise a voltage
        feedback signal. Typically, these inputs are selected when an external
        power meter which generates a voltage output, is connected to the rear
        panel SIG IN connector. In this case the internal amplifier circuit is
        bypassed and the 'Range' bar on the GUI panel is switched off
        (autoranging functionality is not required). Furthermore, although
        tracking occurs as normal, the tracking indicator on the GUI panel is
        inoperative. The INPUT_TIA setting is used when NanoTraking to optimise
        a PIN current feedback signal. The TIA (trans impedence amplifier)
        input source should be selected when using the rear panel OPTICAL/PIN
        I/P connector with either an integral detector, or an external detector
        head connected to the optional SMB adapter. This option uses the
        internal amplifier circuit and associated functionality (e.g.
        autoranging).

        source
            0x01 = P_PZ_NTFBTIA, TIA input
            0x02 = P_PZ_NTFBBNC1V, EXT input (1V range) (N/A for TNA001)
            0x03 = P_PZ_NTFBBNC2V, EXT input (2V range) (N/A for TNA001)
            0x04 = P_PZ_NTFBBNC5V, EXT input (5V range) (N/A for TNA001)
            0x05 = P_PZ_NTFBBNC10V, EXT input (10V range)
        '''
        if source is None:
            # MGMSG_PZ_REQ_NTFEEDBACKSRC 0x063C
            write_buffer = struct.pack("<BBBBBB", 0x3C, 0x06,
                                       0x00, 0x00,
                                       self.dst, self.src)
            self.write(write_buffer)
            # MGMSG_PZ_GET_NTFEEDBACKSRC 0x063D
            read_buffer = self.read(0x3D, 0x06, req_buffer=write_buffer)
            result = struct.unpack("<BBBBBB", read_buffer)
            source = result[2]
            return source
        else:
            assert source in [self.P_PZ_NTFBTIA, self.P_PZ_NTFBBNC10V]
            # MGMSG_PZ_SET_NTFEEDBACKSRC 0x063B
            write_buffer = struct.pack("<BBBBBB", 0x3B, 0x06,
                                       source, 0x00,
                                       self.dst, self.src)
            self.write(write_buffer)
            if persist:
                # MGMSG_NT_SET_EEPROMPARAMS 0x07E7
                write_buffer = struct.pack("<BBBBBBHBB", 0xE7, 0x07,
                                           0x04, 0x00,
                                           self.dst|0x80, self.src,
                                           0x0001, 0x3B, 0x06)
                self.write(write_buffer)

    @_auto_connect
    def status(self):
        '''
        0x00000001  1   Tracking (1 - tracking, 0 - latched).
        0x00000002  2   Tracking with Signal (1 – with signal, 0 – no signal)
        0x00000004  3   Tracking Channel A (1 – Chan A only, 0 – Both channels)
        0x00000008  4   T racking Channel B (1 – Chan B only, 0 – Both channels)
        0x00000010  5   Auto-ranging (1 – auto ranging, 0 manual ranging).
        0x00000020  6   Under Read (1 – under reading, 0 – reading within range).
        0x00000040  7   Over Read (1 – over reading, 0 – reading within range).
        8 to 16     For future use
        0x00010000  17  Channel A Connected (1 – Connected, 0 – Not Connected)
        0x00020000  18  Channel B Connected (1 – Connected, 0 – Not Connected)
        0x00040000  19  Channel A Enabled (1 – Enabled, 0 – Disabled)
        0x00080000  20  Channel B Enabled (1 – Enabled, 0 – Disabled)
        0x00100000  21  Channel A Control Mode (1 – Closed Loop, 0 – Open Loop)
        0x00200000  22  Channel B Control Mode (1 – Closed Loop, 0 – Open Loop)
        23 to 32    For future use
        '''
        #MGMSG_PZ_REQ_NTSTATUSBITS 0x063E
        write_buffer = struct.pack("<BBBBBB", 0x3E, 0x06,
                                   0x01, 0x00,
                                   self.dst, self.src)
        self.write(write_buffer)
        # MGMSG_PZ_GET_NTSTATUSBITS 0x063F
        read_buffer = self.read(0x3F, 0x06, req_buffer=write_buffer)
        result = struct.unpack("<BBBBBBLBBBBBB", read_buffer)
        status_bits = {
            "tracking":     bool(result[6] & 0x00000001),
            "signal":       bool(result[6] & 0x00000002),
            "tracking x":   bool(result[6] & 0x00000004),
            "tracking y":   bool(result[6] & 0x00000008),
            "auto range":   bool(result[6] & 0x00000010),
            "under read":   bool(result[6] & 0x00000020),
            "over read":    bool(result[6] & 0x00000040),
            "x connected":  bool(result[6] & 0x00010000),
            "y connected":  bool(result[6] & 0x00020000),
            "x enabled":    bool(result[6] & 0x00040000),
            "y enabled":    bool(result[6] & 0x00080000),
            "x mode":       bool(result[6] & 0x00100000),
            "y mode":       bool(result[6] & 0x00200000)}
        return status_bits

    @_auto_connect
    def io_settings(self, lv_range=None, lv_route=None, persist=True):
        '''
        LVOutRange
            The output signals from the NanoTrak T-Cube are routed to the piezo
            drivers to position the piezo actuators. Earlier piezo T-cubes
            accept a 5V input while later cubes accept a 10V input. Other piezo
            amplifiers with 5V or 10V input ranges may be driven from the
            NanoTrak T-Cube. This parameter sets the LV output range as
            follows:

            0x01 = RANGE_5V, 0 to 5V output range

            0x02 = RANGE_10V, 0 to 10V output range

        LVOutRoute
            This parameter sets the way the signals are routed to the piezo
            T-Cubes as follows:

            0x01 = RT_SMA, rear panel SMA connectors only

            0x02 = RT_SMA_HUB, rear panel SMA connectors and hub routing
        '''
        if (lv_range is None) and (lv_route is None):
            # MGMSG_NT_REQ_TNAIOSETTINGS 0x07EC
            write_buffer = struct.pack("<BBBBBB", 0xEC, 0x07,
                                       0x01, 0x00,
                                       self.dst, self.src)
            self.write(write_buffer)
            # MGMSG_NT_GET_TNAIOSETTINGS 0x07ED
            read_buffer = self.read(0xED, 0x07, req_buffer=write_buffer)
            result = struct.unpack("<BBBBBBHHHH", read_buffer)
            lv_out_range = result[6]
            lv_out_route = result[7]
            self._lv_range = lv_out_range
            self._lv_route = lv_out_route
            return {"lv_range":lv_out_range, "lv_route":lv_out_route}
        else:
            if lv_range is None:
                lv_range = self._lv_range
            if lv_route is None:
                lv_route = self._lv_route
            assert lv_range in [self.RANGE_5V, self.RANGE_10V]
            assert lv_route in [self.RT_SMA, self.RT_SMA_HUB]
            # MGMSG_NT_SET_TNAIOSETTINGS 0x07EB
            write_buffer = struct.pack("<BBBBBBHHHH", 0xEB, 0x07,
                                       0x08, 0x00,
                                       self.dst|0x80, self.src,
                                       lv_range,
                                       lv_route,
                                       0x000, 0x000)
            self.write(write_buffer)
            self._lv_range = lv_range
            self._lv_route = lv_route
            if persist:
                # MGMSG_NT_SET_EEPROMPARAMS 0x07E7
                write_buffer = struct.pack("<BBBBBBHBB", 0xE7, 0x07,
                                           0x04, 0x00,
                                           self.dst|0x80, self.src,
                                           0x0001, 0xEB, 0x07)
                self.write(write_buffer)
