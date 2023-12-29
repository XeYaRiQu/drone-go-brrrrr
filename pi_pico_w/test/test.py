# -*- coding: utf-8 -*-
"""

"""

##### Imports #####

import time
from machine import UART, Pin, PWM, freq, I2C


##### Classes #####

class IBus:
    """
    A class to handle iBus signals received from UART.

    An iBus packet comprises 32 bytes:
        - 2 header bytes (first header is 0x20, second is 0x40)
        - 28 channel bytes (2 bytes per channel value)
        - 2 checksum bytes (1 checksum value)

    The protocol data rate is 115200 baud. A packet is transmitted every 7 ms.
    Each packet takes 32*8/115200 seconds to transmit (about 2.22 ms).
    Checksum = 0xFFFF - sum of first 30 bytes
    The channels' values are transmitted sequentially in little endian byte order.
    """

    max_read_attempts = 4
    number_of_channels = 6

    def __init__(self, uart_number):
        self.uart = UART(id=uart_number)
        self.uart.init(baudrate=115200, bits=8, parity=None, stop=1)
        self.channel_values = [0] * self.number_of_channels

    def read(self, buffer):
        """
        The UART FIFO buffer holds data until it is cleared by
        a UART read. This means that additional data is dropped.
        If a read fails, try to attempt to read the next packet.
        """

        for attempt in range(self.max_read_attempts):
            buffer = bytearray(32)
            self.uart.readinto(buffer, 32)

            # Validate start bytes
            if buffer[0] == 0x20 and buffer[1] == 0x40:
                print("DEBUG >>>>    Packet received, decoding.")
                checksum = 0xFF9F # 0xFFFF - 0x20 - 0x40

                for byte_index in range(2, 33):
                    checksum -= buffer[byte_index]

                # Validate checksum
                if checksum == (buffer[30] << 8) | buffer[29]: # Converting to big endian
                    print("DEBUG >>>>   Checksum passed.")

                    for channel in range(self.number_of_channels):
                        self.channel_values[channel] = (buffer[channel*2 + 3] << 8) + buffer[channel*2 + 2]:

                    return self.channel_values
                else:
                    print("DEBUG >>>>   Checksum failed.")
            else:
                print("DEBUG >>>>   Start bytes incorrect. Retrying: ", attempt)

        print("DEBUG >>>>   No successful packet received.")
        return self.channel_values


class Mpu6050(I2C):
    """
    A class to handle I2C data from IMU.
    """
    PWR_MGMT_1 = 0x6B
    SMPLRT_DIV = 0x19
    CONFIG = 0x1A
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C
    TEMP_OUT_H = 0x41
    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H = 0x43

    def init_mpu6050(self, address=0x68):
        self.writeto_mem(address, PWR_MGMT_1, b'\x00')
        time.sleep_ms(100)
        self.writeto_mem(address, SMPLRT_DIV, b'\x07')
        self.writeto_mem(address, CONFIG, b'\x00')
        self.writeto_mem(address, GYRO_CONFIG, b'\x00')
        self.writeto_mem(address, ACCEL_CONFIG, b'\x00')
    
    def read_raw_data(i2c, addr, address=0x68):
        high = i2c.readfrom_mem(address, addr, 1)[0]
        low = i2c.readfrom_mem(address, addr + 1, 1)[0]
        value = high << 8 | low
        if value > 32768:
            value = value - 65536
        return value
    
    def get_mpu6050_data(i2c):
        temp = read_raw_data(i2c, TEMP_OUT_H) / 340.0 + 36.53
        accel_x = read_raw_data(i2c, ACCEL_XOUT_H) / 16384.0
        accel_y = read_raw_data(i2c, ACCEL_XOUT_H + 2) / 16384.0
        accel_z = read_raw_data(i2c, ACCEL_XOUT_H + 4) / 16384.0
        gyro_x = read_raw_data(i2c, GYRO_XOUT_H) / 131.0
        gyro_y = read_raw_data(i2c, GYRO_XOUT_H + 2) / 131.0
        gyro_z = read_raw_data(i2c, GYRO_XOUT_H + 4) / 131.0
    
        return {
            'temp': temp,
            'accel': {
                'x': accel_x,
                'y': accel_y,
                'z': accel_z,
            },
            'gyro': {
                'x': gyro_x,
                'y': gyro_y,
                'z': gyro_z,
            }
        }


##### Pin settings #####

pin_led = Pin("LED", Pin.OUT)
pin_ibus = IBus(1)


##### Definitions #####

def setup() -> None:
    print("INFO >>>>    Starting setup sequence.")

    # Flash LED for 2 seconds
    for i in range(20):
        pin_led.toggle()
        time.sleep_ms(100)
    pin_led.off()

    freq(250000000)
    print("INFO >>>>    RP2040 overclocked to 250 MHz.")


def main() -> None:
    while True:
        pass

        


##### Main #####


