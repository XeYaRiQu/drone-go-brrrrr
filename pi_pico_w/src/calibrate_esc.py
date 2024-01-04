# -*- coding: utf-8 -*-
"""
Ensure this script is running before performing any calibration steps.

To calibrate throttle range:
    1. Push throttle stick to highest position
    2. Connect battery to ESCs
    3. Wait for 2 beeps
    4. Immediately push the throttle stick to lowest position
    5. Calibration completed upon '1 2 3' beeping tune

To reset to factory settings:
    1. Push throttle stick to highest position
    2. Connect battery to ESCs
    3. Wait for 2 beeps, followed by '1 2 3 1 2 3' beeping tune.
    4. Immediately push the throttle stick to lowest position
    5. Calibration completed upon '1 2 3' beeping tune

The ESCs uses Morse code to indicate numbers.
        NUMBERS      MORSE
           1           .
           2           ..
           3           ...
           4           ....
           5           -
           6           -.
           7           -..
           8           -...
           9           -....
          10           --

Settings available:
    a) Brake type
        1) Off (default)
        2) On

    b) Timing mode
        1) Low:      0 deg
        2) Mid-low:  8 deg
        3) Middle:   15 deg (default)
        4) Mid-high: 23 deg
        5) High:     30 deg

    c) Start force:
        1) 0.031
        2) 0.047
        3) 0.063
        4) 0.094
        5) 0.125
        6) 0.188
        7) 0.250
        8) 0.380
        9) 0.500
       10) 0.750 (default)
       11) 1.000
       12) 1.250
       13) 1.500

    d) Curve mode
        1) Off (default)
        2) Low
        3) Middle
        4) High

    e) Control frequency
        1) 8 kHz (default)
        2) 22 kHz

    f) Low voltage protection
        1) Off
        2) 2.8 V/cell
        3) 3.0 V/cell (default)
        4) 3.2 V/cell

    g) Cutoff mode
        1) Soft cut (default)
        2) Cut off

    h) Rotation direction
        1) Normal (default)
        2) Reverse
        3) Bi-directional
"""

##### Imports #####

import time
from machine import Pin, UART, PWM, freq


##### Constants #####

RC_THROTTLE_CH = 2
RC_ROLL_CH = 0
RC_PITCH_CH = 1
RC_YAW_CH = 3
RC_EXTRA1_CH = 4
RC_EXTRA2_CH = 5


##### Pin settings #####

LED_GPIO = Pin("LED", Pin.OUT)

RC_GPIO_IN = Pin(5)
RC_GPIO_OUT = Pin(4)

MOTOR1_GPIO = Pin(2)
MOTOR2_GPIO = Pin(28)
MOTOR3_GPIO = Pin(15)
MOTOR4_GPIO = Pin(16)


##### Enable pin IO #####

rc:UART = UART(1)

motor1:PWM = PWM(MOTOR1_GPIO)
motor2:PWM = PWM(MOTOR2_GPIO)
motor3:PWM = PWM(MOTOR3_GPIO)
motor4:PWM = PWM(MOTOR4_GPIO)


##### Others #####

raw_rc_values:list[int] = [0]*6


##### Functions #####

def rc_read() -> None:
    """
    An iBus packet comprises 32 bytes:
        - 2 header bytes (first header is 0x20, second is 0x40)
        - 28 channel bytes (2 bytes per channel value)
        - 2 checksum bytes (1 checksum value)

    The protocol data rate is 115200 baud. A packet is transmitted every 7 ms.
    Each packet takes 32*8/115200 seconds to transmit (about 2.22 ms).
    Checksum = 0xFFFF - sum of first 30 bytes
    The channels' values are transmitted sequentially in little endian byte order.
    """

    for attempt in range(6):
        if rc.any():
            buffer = bytearray(30)
            char1 = rc.read(1)
            char2 = rc.read(1)

            # Validate start bytes
            if char1 == b'\x20' and char2 == b'\x40':
                rc.readinto(buffer)
                checksum = 0xFF9F # = 0xFFFF - 0x20 - 0x40 or 65439 in decimal

                for byte_index in range(28):
                    checksum -= buffer[byte_index]

                # Validate checksum
                if checksum == (buffer[29] << 8) | buffer[28]: # Converting to big endian
                    for channel in range(6):
                        raw_rc_values[channel] = (buffer[channel*2 + 1] << 8) + buffer[channel*2]

                    break


##### Main #####

rc.init(baudrate=115200, bits=8, parity=None, stop=1, rx=RC_GPIO_IN)
motor1.freq(250)
motor2.freq(250)
motor3.freq(250)
motor4.freq(250)

while True:
    rc_read()
    motor1.duty_ns(raw_rc_values[RC_THROTTLE_CH]*1000)
    motor2.duty_ns(raw_rc_values[RC_THROTTLE_CH]*1000)
    motor3.duty_ns(raw_rc_values[RC_THROTTLE_CH]*1000)
    motor4.duty_ns(raw_rc_values[RC_THROTTLE_CH]*1000)
    print(raw_rc_values)