# -*- coding: utf-8 -*-

##### Imports #####

import time
from machine import UART, Pin

##### Classes and variables #####

class IBus():
    """
    A class to handle iBus signals received from UART.

    An iBus packet comprises 32 bytes:
        - 2 header bytes (first header is 0x20, second is 0x40)
        - 28 channel bytes (2 bytes per channel, little endian)
        - 2 checksum bytes (1 checksum value, little endian)

    The default data rate is 115200 baud. A packet is transmitted every 7 ms.
    Checksum = 0xFFFF - sum of first 30 bytes
    """

    def __init__(self, uart_number, baud = 115200, num_channels = 6):
        self.uart = UART(uart_number, baud)
        self.num_channels = num_channels

        self.ch = [0]*self.num_channels

    # Returns list with raw data
    def read(self):

        buffer = bytearray(30)

        # Attempt to catch a packet 10 times
        for attempt in range(10):
            header1 = self.uart.read(1) # Read the first byte from UART
            header2 = self.uart.read(1) # Read the second byte from UART

            if header1 == 0x20 and header2 == 0x40: # Checking for correct header bytes
                self.uart.readinto(buffer) # Store the rest of the packet into buffer

                checksum = 0xFF9F # 0xFFFF - 0x20 - 0x40
                # Validate the checksum
                for byte_number in range(29):
                    checksum -= buffer[byte_number]

                if checksum == (buffer[30] << 8) | buffer[29]: # Comparing the calculated checksum against the sent value
                    # buffer[0] = 0x40
                    # self.ch[0] = 1 # status 1 = success
                    for channel in range (self.num_channels):
                        self.ch[channel] = (buffer[channel*2] + (buffer[channel*2 + 1] << 8))                    
                    return self.ch
                else:
                    # Checksum error
                    self.ch[0] = -2
            else:
                self.ch[0] = -1
                
        # Reach here then timed out
        self.ch[0] = -1
        return self.ch
    
    
    # Convert to meaningful values - eg. -100 to 100
    # Typical use for FS-iA6B
    # channel 1 to 4 use type="default" provides result from -100 to +100 (0 in centre)
    # channel 5 & 6 are dials type="dial" provides result from 0 to 100 
    # Note approx depends upon calibration etc.
    @staticmethod
    def normalize (value, type="default"):
        if (type == "dial"):
            return ((value - 1000) / 10)
        else:
            return ((value - 1500) / 5)
        
Pin("LED", Pin.OUT).on()

ibus_in = IBus(1)

while True:
    res = ibus_in.read()
    # if signal then display immediately
    if (res[0]):
        print ("Ch.1: {:<6}  Ch.2 {:<6}  Ch.3 {:<6}  Ch.4 {:<6}  Ch.5 {:<6}  Ch.6 {:<6}".format(
            IBus.normalize(res[1]),
            IBus.normalize(res[2]),
            IBus.normalize(res[3]),
            IBus.normalize(res[4]),
            IBus.normalize(res[5], type="dial"),
            IBus.normalize(res[6], type="dial"),
            end=""))
