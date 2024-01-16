# -*- coding: utf-8 -*-
"""
Avoid using classes and minimise unnecessary function calls
to increase processing speed in the main flight control loop.
Try to replace integer division with float multiplication.
"""

##### Imports #####

import time
from machine import Pin, I2C, freq


##### Pin settings #####

LED_GPIO = Pin("LED", Pin.OUT)

IMU_GPIO_SDA = Pin(12, pull=Pin.PULL_UP)
IMU_GPIO_SCL = Pin(13, pull=Pin.PULL_UP)


##### Constants #####

IMU_I2C_ADDRESS = 104      # 0x68
IMU_REG_SMPLRT_DIV = 25    # 0x19
IMU_REG_SIGNAL_RESET = 104 # 0x68
IMU_REG_PWR_MGMT1 = 107    # 0x6B
IMU_REG_WHO_AM_I = 117     # 0x75
IMU_REG_CONFIG = 26        # 0x1A
IMU_REG_GYRO_CONFIG = 27   # 0x1B
IMU_REG_ACCE_CONFIG = 28   # 0x1C
IMU_REG_ACCE_X_HI = 59     # 0x3B
IMU_REG_ACCE_X_LO = 60     # 0x3C
IMU_REG_ACCE_Y_HI = 61     # 0x3D
IMU_REG_ACCE_Y_LO = 62     # 0x3E
IMU_REG_ACCE_Z_HI = 63     # 0x3F
IMU_REG_ACCE_Z_LO = 64     # 0x40
IMU_REG_TEMP_HI = 65       # 0x41
IMU_REG_TEMP_LO = 66       # 0x42
IMU_REG_GYRO_X_HI = 67     # 0x43
IMU_REG_GYRO_X_LO = 68     # 0x44
IMU_REG_GYRO_Y_HI = 69     # 0x45
IMU_REG_GYRO_Y_LO = 70     # 0x46
IMU_REG_GYRO_Z_HI = 71     # 0x47
IMU_REG_GYRO_Z_LO = 72     # 0x48


##### Enable pin IO #####

imu:I2C = I2C(0, sda=IMU_GPIO_SDA, scl=IMU_GPIO_SCL)
led=LED_GPIO


##### Functions #####

x = int.from_bytes(imu.readfrom_mem(IMU_I2C_ADDRESS, IMU_REG_WHO_AM_I, 1), 'big')
print("INFO  >>>>   Who am I value:", x)
x = int.from_bytes(imu.readfrom_mem(IMU_I2C_ADDRESS, IMU_REG_PWR_MGMT1, 1), 'big')
print("INFO  >>>>   IMU state:", x)

print("INFO  >>>>   Waking IMU")
imu.writeto_mem(IMU_I2C_ADDRESS, IMU_REG_PWR_MGMT1, bytearray(b'\x0b'))
time.sleep_ms(3000)
x = int.from_bytes(imu.readfrom_mem(IMU_I2C_ADDRESS, IMU_REG_PWR_MGMT1, 1), 'big')
print("INFO  >>>>   IMU state:", x)

# print("INFO  >>>>   Resetting all registers in MPU-6050")
# imu.writeto_mem(IMU_I2C_ADDRESS, IMU_REG_PWR_MGMT1, bytes(128)) # Reset all registers
# time.sleep_ms(100)

# print("INFO  >>>>   Resetting all sensor signal paths")
# imu.writeto_mem(IMU_I2C_ADDRESS, IMU_REG_SIGNAL_RESET, bytes(7)) # Reset analog and digital paths for all sensors
# time.sleep_ms(100)


# x = int.from_bytes(imu.readfrom_mem(IMU_I2C_ADDRESS, 65, 1), 'big')
# print("INFO  >>>>   Register 65 value set to:", x)
# x = int.from_bytes(imu.readfrom_mem(IMU_I2C_ADDRESS, 66, 1), 'big')
# print("INFO  >>>>   Register 66 value set to:", x)

# print("INFO  >>>>   Setting power management with value: 0")
# imu.writeto_mem(IMU_I2C_ADDRESS, IMU_REG_PWR_MGMT1, bytes(0))   # Wake, disable temperature sensor
# time.sleep_ms(100)
# x = int.from_bytes(imu.readfrom_mem(IMU_I2C_ADDRESS, IMU_REG_PWR_MGMT1, 1), 'big')
# print("INFO  >>>>   Register value set to:", x)

# print("INFO  >>>>   Setting sensor sample rate with value: 3")
# imu.writeto_mem(IMU_I2C_ADDRESS, IMU_REG_SMPLRT_DIV, bytes(3))  # Set sensor sample rate to 250 Hz
# time.sleep_ms(100)
# x = int.from_bytes(imu.readfrom_mem(IMU_I2C_ADDRESS, IMU_REG_SMPLRT_DIV, 1), 'big')
# print("INFO  >>>>   Register value set to:", x)

# print("INFO  >>>>   Setting gyroscope scale with value: 8")
# imu.writeto_mem(IMU_I2C_ADDRESS, IMU_REG_GYRO_CONFIG, bytes(8)) # Set gyroscope scale to 500 dps
# time.sleep_ms(100)
# x = int.from_bytes(imu.readfrom_mem(IMU_I2C_ADDRESS, IMU_REG_GYRO_CONFIG, 1), 'big')
# print("INFO  >>>>   Register value set to:", x)

# print("INFO  >>>>   Setting LPF with value: 3")
# imu.writeto_mem(IMU_I2C_ADDRESS, IMU_REG_CONFIG, bytes(3))      # Set accelerometer LPF to 44 Hz and gyroscope LPF to 42 Hz
# time.sleep_ms(100)
# x = int.from_bytes(imu.readfrom_mem(IMU_I2C_ADDRESS, IMU_REG_CONFIG, 1), 'big')
# print("INFO  >>>>   Register value set to:", x)

# x = int.from_bytes(imu.readfrom_mem(IMU_I2C_ADDRESS, 65, 1), 'big')
# print("INFO  >>>>   Register 65 value set to:", x)
# x = int.from_bytes(imu.readfrom_mem(IMU_I2C_ADDRESS, 66, 1), 'big')
# print("INFO  >>>>   Register 66 value set to:", x)