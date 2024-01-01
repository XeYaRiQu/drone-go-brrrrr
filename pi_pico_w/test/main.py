# -*- coding: utf-8 -*-
"""
Avoid using classes and minimise unnecessary function calls
to increase processing speed in the main flight control loop.
"""

##### Imports #####

import time
from machine import UART, Pin, PWM, freq, I2C


##### General settings #####

# 250, 500, 1000, 2000 dps (IMU_REG_GYRO_CONFIG[3:4])
gyro_range:int = 500
# 2, 4, 8, 16 g (IMU_REG_ACCE_CONFIG[3:4])
acce_range:int = 4


##### Constants #####

RC_THROTTLE_CH = 2
RC_ROLL_CH = 0
RC_PITCH_CH = 1
RC_YAW_CH = 3
RC_EXTRA1_CH = 4
RC_EXTRA2_CH = 5

IMU_I2C_ADDRESS = 104    # 0x68
IMU_REG_SMPLRT_DIV = 25  # 0x19
IMU_REG_PWR_MGMT1 = 107  # 0x6B
IMU_REG_WHO_AM_I = 117   # 0x75

IMU_REG_CONFIG = 26      # 0x1A
IMU_REG_GYRO_CONFIG = 27 # 0x1B
IMU_REG_ACCE_CONFIG = 28 # 0x1C
IMU_REG_ACCE_X_HI = 59   # 0x3B
IMU_REG_ACCE_X_LO = 60   # 0x3C
IMU_REG_ACCE_Y_HI = 61   # 0x3D
IMU_REG_ACCE_Y_LO = 62   # 0x3E
IMU_REG_ACCE_Z_HI = 63   # 0x3F
IMU_REG_ACCE_Z_LO = 64   # 0x40
IMU_REG_TEMP_HI = 65     # 0x41
IMU_REG_TEMP_LO = 66     # 0x42
IMU_REG_GYRO_X_HI = 67   # 0x43
IMU_REG_GYRO_X_LO = 68   # 0x44
IMU_REG_GYRO_Y_HI = 69   # 0x45
IMU_REG_GYRO_Y_LO = 70   # 0x46
IMU_REG_GYRO_Z_HI = 71   # 0x47
IMU_REG_GYRO_Z_LO = 72   # 0x48

# The scale modifiers change with respect to the scale settings
# used for the IMU's accelerometer and gyroscope.
# The 16-bit 2's complement readings range from -32768 to 32767
# The measurement range depends on the bits written to IMU_REG_GYRO_CONFIG
# The modifer converts raw values back into physical measurements
SCALE_MODIFIER_ACCE:float = 32767/acce_range
SCALE_MODIFIER_GYRO:float = 32767/gyro_range


##### Pin settings #####

LED_GPIO = Pin("LED", Pin.OUT)

RC_GPIO_IN = Pin(5)
RC_GPIO_OUT = Pin(4)

IMU_GPIO_SDA = Pin(12, pull=Pin.PULL_UP)
IMU_GPIO_SCL = Pin(13, pull=Pin.PULL_UP)

MOTOR1_GPIO = Pin(2)
MOTOR2_GPIO = Pin(28)
MOTOR3_GPIO = Pin(15)
MOTOR4_GPIO = Pin(16)


##### Control variables #####

raw_rc_values:list = [0]*6
normalised_rc_values = [0]*6

normalised_gyro_values = [0.0]*3

roll_last_integral:float = 0.0
roll_last_error:float = 0.0
pitch_last_integral:float = 0.0
pitch_last_error:float = 0.0
yaw_last_integral:float = 0.0
yaw_last_error:float = 0.0

pid_roll_kp:float = 0.00043714285
pid_roll_ki:float = 0.00255
pid_roll_kd:float = 0.00002571429
pid_pitch_kp:float = pid_roll_kp
pid_pitch_ki:float = pid_roll_ki
pid_pitch_kd:float = pid_roll_kd
pid_yaw_kp:float = 0.001714287
pid_yaw_ki:float = 0.003428571
pid_yaw_kd:float = 0.0

max_rate_roll:float = 30.0
max_rate_pitch:float = 30.0
max_rate_yaw:float = 50.0


##### Enable pin IO #####

rc:UART = UART(1)
imu:I2C = I2C(0, sda=IMU_GPIO_SDA, scl=IMU_GPIO_SCL)
motor1:PWM = PWM(MOTOR1_GPIO)
motor2:PWM = PWM(MOTOR2_GPIO)
motor3:PWM = PWM(MOTOR3_GPIO)
motor4:PWM = PWM(MOTOR4_GPIO)


##### Others #####

error_list:list = []


##### Functions #####

def setup() -> int:
    error_raised_flag:bool = False

    print("INFO >>>>    Starting setup sequence.\n")

    # Flash LED for 5 seconds
    for i in range(5):
        LED_GPIO.toggle()
        time.sleep_ms(500)
        LED_GPIO.toggle()
        time.sleep_ms(500)
    LED_GPIO.off()

    try:
        freq(250000000)
        print("INFO >>>>    Overclock RP2040 to 250 MHz --> SUCCESS.\n")
    except:
        error_raised_flag = True
        error_list.append("Unable to overclock RP2040.")
        print("ERROR >>>>   Overclock RP2040 to 250 MHz --> FAIL\n")

    try:
        rc.init(baudrate=115200, bits=8, parity=None, stop=1, rx=RC_GPIO_IN)
        print("INFO >>>>    RC receiver setup --> SUCCESS\n")
    except:
        error_raised_flag = True
        error_list.append("Unable to initialise UART for RC receiver.")
        print("ERROR >>>>   RC receiver setup --> FAIL\n")

    try:
        imu.writeto_mem(IMU_I2C_ADDRESS, IMU_REG_PWR_MGMT1, bytes(1))   # Wake command
        imu.writeto_mem(IMU_I2C_ADDRESS, IMU_REG_CONFIG, bytes(5))      # Set accelerometer LPF to 10 Hz
        imu.writeto_mem(IMU_I2C_ADDRESS, IMU_REG_GYRO_CONFIG, bytes(8)) # Set gyroscope scale to 500 dps
        print("INFO >>>>    MPU-6050 setup --> SUCCESS\n")
    except:
        # Masking the fail flag here as IMU wiring needs some work
        # IMU SDA and SCL must to be connected to strong pull-up resistors
        # error_raised_flag = True
        error_list.append("IMU I2C write error.")
        print("ERROR >>>>   MPU-6050 setup --> FAIL\n")

    try:
        # Who am I check
        if imu.readfrom_mem(IMU_I2C_ADDRESS, IMU_REG_WHO_AM_I, 1)[0] == IMU_I2C_ADDRESS:
            print("INFO >>>>   MPU-6050 verify WHO_AM_I -> SUCCESS")
        else:
            error_raised_flag = True
            error_list.append("MPU-6050 WHO_AM_I read error.")
            print("ERROR >>>>  MPU-6050 verify WHO_AM_I -> FAIL")

        # Low pass filter check
        if imu.readfrom_mem(IMU_I2C_ADDRESS, IMU_REG_CONFIG, 1)[0] == 5:
            print("INFO >>>>    MPU-6050 verify DLPF -> SUCCESS")
        else:
            error_raised_flag = True
            error_list.append("MPU-6050 DLPF not set.")
            print("ERROR >>>>  MPU-6050 verify DLPF -> FAIL")
    except:
        error_raised_flag = True
        error_list.append("IMU I2C read error.")
        print("ERROR >>>>  MPU-6050 verify settings -> FAIL")

    try:
        motor1.freq(250)
        motor2.freq(250)
        motor3.freq(250)
        motor4.freq(250)
        print("INFO >>>>    Set motor PWM freq to 250 Hz --> SUCCESS\n")
    except:
        error_raised_flag = True
        error_list.append("Unable to set motor PWM freq to 250 Hz.")
        print("ERROR >>>>   Set motor PWM freq to 250 Hz --> FAIL\n")

    if not error_raised_flag:
        return 0
    else:
        return 1


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

    # Normalise data
    for index in range(6):
        # normalised_rc_values[index] = float(raw_rc_values[index]/1000 - 1) # From 1000-2000 to 0.0-1.0
        normalised_rc_values[index] = raw_rc_values[index]*1000 # From 1000-2000 to 1000000-2000000


def imu_read() -> None:
    """
    The IMU measurements are 16-bit 2's complement values.
    """
    raw_gyro_data = imu.readfrom_mem(IMU_I2C_ADDRESS, IMU_REG_GYRO_X_HI, 6)

    for axis in range(3):
        value = (raw_gyro_data[axis*2] << 8) | raw_gyro_data[axis*2 + 1]

        # Check if negative
        if raw_gyro_data[axis*2] >= 0xFF:
            normalised_gyro_values[axis] = -((0xFFFF - value) + 1) / SCALE_MODIFIER_GYRO
        else:
            normalised_gyro_values[axis] = value / SCALE_MODIFIER_GYRO


##### Main #####

if setup() == 0:
    while True:
        # start_time = time.ticks_us()

        try:
            rc_read()
            print(normalised_rc_values)

            # Just simple throttle testing for now
            motor1.duty_ns(normalised_rc_values[RC_THROTTLE_CH])
            motor2.duty_ns(normalised_rc_values[RC_THROTTLE_CH])
            motor3.duty_ns(normalised_rc_values[RC_THROTTLE_CH])
            motor4.duty_ns(normalised_rc_values[RC_THROTTLE_CH])

            # imu_read()
            desired_throttle:float = normalised_rc_values[RC_THROTTLE_CH]
            desired_pitch:float = normalised_rc_values[RC_PITCH_CH]
            desired_roll:float = normalised_rc_values[RC_ROLL_CH]
            desired_yaw:float = normalised_rc_values[RC_YAW_CH]
        except:
            break

        # print("INFO >>>>    Loop time:", time.ticks_diff(time.ticks_us(), start_time))
else:
    print("ERROR >>>   Setup failed.\n")

    for error in error_list:
        print("ERROR >>>>  ", error)
