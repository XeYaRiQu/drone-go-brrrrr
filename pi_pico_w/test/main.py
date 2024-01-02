# -*- coding: utf-8 -*-
"""
Avoid using classes and minimise unnecessary function calls
to increase processing speed in the main flight control loop.
Try to replace integer division with float multiplication.
"""

##### Imports #####

import time
from machine import Pin, UART, I2C, PWM, freq


##### General settings #####

gyro_range:int = 500 # 250, 500, 1000, 2000 dps (IMU_REG_GYRO_CONFIG[3:4])
acce_range:int = 4   # 2, 4, 8, 16 g (IMU_REG_ACCE_CONFIG[3:4])

min_throttle_rate:float = 0.10  # Maximum throttle at 0% input (does not generate thrust)
max_throttle_rate:float = 0.40  # Maximum throttle at 100% input (limits acceleration)


##### PID settings #####

max_roll_rate:float = 30.0  # degrees per second
max_pitch_rate:float = 30.0 # degrees per second
max_yaw_rate:float = 60.0   # degrees per second

pid_kp_roll:float = 0.00043714285
pid_kp_pitch:float = 0.00043714285
pid_kp_yaw:float = 0.001714287

pid_ki_roll:float = 0.00255
pid_ki_pitch:float = 0.00255
pid_ki_yaw:float = 0.003428571

pid_kd_roll:float = 0.00002571429
pid_kd_pitch:float = 0.00002571429
pid_kd_yaw:float = 0.0

pid_integral_limit_pos:float = 100.0
pid_integral_limit_neg:float = -pid_integral_limit_pos


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

GYRO_INDEX_ROLL = 0
GYRO_INDEX_PITCH = 1
GYRO_INDEX_YAW = 2

SCALE_MULTIPLIER_ACCE:float = acce_range / 32767
SCALE_MULTIPLIER_GYRO:float = gyro_range / 32767

THROTTLE_RANGE = max_throttle_rate - min_throttle_rate

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


##### Enable pin IO #####

rc:UART = UART(1)
imu:I2C = I2C(0, sda=IMU_GPIO_SDA, scl=IMU_GPIO_SCL)
motor1:PWM = PWM(MOTOR1_GPIO)
motor2:PWM = PWM(MOTOR2_GPIO)
motor3:PWM = PWM(MOTOR3_GPIO)
motor4:PWM = PWM(MOTOR4_GPIO)


##### Others #####

error_list:list = []

raw_rc_values:list = [0] * 6
normalised_rc_values:list = [0] * 6
normalised_gyro_values:list = [0.0] * 3

# This are the starting values that will be used in the first PID calculation
prev_pid_error_roll:float = 0.0
prev_pid_error_pitch:float = 0.0
prev_pid_error_yaw:float = 0.0
prev_pid_inte_roll:float = 0.0
prev_pid_inte_pitch:float = 0.0
prev_pid_inte_yaw:float = 0.0


##### Functions #####

def setup() -> int:
    error_raised_flag:bool = False

    print("INFO  >>>>   Starting setup sequence.\n")

    # Flash LED for 5 seconds
    for i in range(5):
        LED_GPIO.toggle()
        time.sleep_ms(500)
        LED_GPIO.toggle()
        time.sleep_ms(500)
    LED_GPIO.off()

    try:
        freq(250000000)
        print("INFO  >>>>   Overclock RP2040 to 250 MHz --> SUCCESS.\n")
    except:
        error_raised_flag = True
        error_list.append("Unable to overclock RP2040.")
        print("ERROR >>>>   Overclock RP2040 to 250 MHz --> FAIL\n")

    try:
        rc.init(baudrate=115200, bits=8, parity=None, stop=1, rx=RC_GPIO_IN)
        print("INFO  >>>>   RC receiver setup --> SUCCESS\n")
    except:
        error_raised_flag = True
        error_list.append("Unable to initialise UART for RC receiver.")
        print("ERROR >>>>   RC receiver setup --> FAIL\n")

    try:
        imu.writeto_mem(IMU_I2C_ADDRESS, IMU_REG_PWR_MGMT1, bytes(1))   # Wake command
        imu.writeto_mem(IMU_I2C_ADDRESS, IMU_REG_CONFIG, bytes(5))      # Set accelerometer LPF to 10 Hz
        imu.writeto_mem(IMU_I2C_ADDRESS, IMU_REG_GYRO_CONFIG, bytes(8)) # Set gyroscope scale to 500 dps
        print("INFO  >>>>   MPU-6050 setup --> SUCCESS\n")
    except:
        # Masking the fail flag here as IMU wiring needs some work
        # IMU SDA and SCL must to be connected to strong pull-up resistors
        # error_raised_flag = True
        error_list.append("IMU I2C write error.")
        print("ERROR >>>>   MPU-6050 setup --> FAIL\n")

    try:
        # Who am I check
        if imu.readfrom_mem(IMU_I2C_ADDRESS, IMU_REG_WHO_AM_I, 1)[0] == IMU_I2C_ADDRESS:
            print("INFO  >>>>   MPU-6050 verify WHO_AM_I -> SUCCESS\n")
        else:
            error_raised_flag = True
            error_list.append("MPU-6050 WHO_AM_I read error.")
            print("ERROR >>>>   MPU-6050 verify WHO_AM_I -> FAIL\n")

        # Low pass filter check
        if imu.readfrom_mem(IMU_I2C_ADDRESS, IMU_REG_CONFIG, 1)[0] == 5:
            print("INFO  >>>>   MPU-6050 verify DLPF -> SUCCESS\n")
        else:
            error_raised_flag = True
            error_list.append("MPU-6050 DLPF not set.")
            print("ERROR >>>>   MPU-6050 verify DLPF -> FAIL\n")
    except:
        # Masking the fail flag here as IMU wiring needs some work
        # IMU SDA and SCL must to be connected to strong pull-up resistors
        # error_raised_flag = True
        error_list.append("IMU I2C read error.")
        print("ERROR >>>>   MPU-6050 verify settings -> FAIL\n")

    try:
        motor1.freq(250)
        motor2.freq(250)
        motor3.freq(250)
        motor4.freq(250)
        print("INFO  >>>>   Set motor PWM freq to 250 Hz --> SUCCESS\n")
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
                        raw_rc_values[channel] = (buffer[channel*2 + 1] << 8) + buffer[channel * 2]

                    break

    # Normalise data
    for index in range(6):
        # Normalise to 0.0-1.0 as these values are scaled with respect to the maximum rates defined in settings
        # normalised_rc_values[index] = float(raw_rc_values[index]*0.001 - 1) # From 1000-2000 to 0.0-1.0
        normalised_rc_values[index] = raw_rc_values[index] * 1000 # From 1000-2000 to 1000000-2000000 for simple testing


def imu_read() -> None:
    """
    The IMU measurements are 16-bit 2's complement values ranging from -32768
    to 32767 which needs to be divided by the scale multipliers to obtain the
    physical values. These scale multipliers change depending on the range set
    in their respective register configs. The high byte is read before the low
    byte for each measurement.
    """
    raw_gyro_data = imu.readfrom_mem(IMU_I2C_ADDRESS, IMU_REG_GYRO_X_HI, 6)

    for axis in range(3):
        value = (raw_gyro_data[axis * 2] << 8) | raw_gyro_data[axis*2 + 1]

        # Check if negative
        if raw_gyro_data[axis * 2] >= 128:
            normalised_gyro_values[axis] = -((0xFFFF - value) + 1) * SCALE_MULTIPLIER_GYRO
        else:
            normalised_gyro_values[axis] = value * SCALE_MULTIPLIER_GYRO


##### Main #####

if setup() == 0:
    prev_pid_timestamp:int = time.ticks_us()
    print("INFO  >>>>   Starting flight control loop.\n")

    while True:
        try:
            rc_read()
            # imu_read()
            print(normalised_rc_values)

            # Just simple throttle testing for now
            motor1.duty_ns(normalised_rc_values[RC_THROTTLE_CH])
            motor2.duty_ns(normalised_rc_values[RC_THROTTLE_CH])
            motor3.duty_ns(normalised_rc_values[RC_THROTTLE_CH])
            motor4.duty_ns(normalised_rc_values[RC_THROTTLE_CH])

            desired_throttle_rate:float = normalised_rc_values[RC_THROTTLE_CH]
            desired_pitch_rate:float = normalised_rc_values[RC_PITCH_CH]
            desired_roll_rate:float = normalised_rc_values[RC_ROLL_CH]
            desired_yaw_rate:float = normalised_rc_values[RC_YAW_CH]

            # Error calculations (desired - actual)
            pid_error_roll:float = desired_roll_rate*max_roll_rate - normalised_gyro_values[GYRO_INDEX_ROLL]
            pid_error_pitch:float = desired_pitch_rate*max_pitch_rate - normalised_gyro_values[GYRO_INDEX_PITCH]
            pid_error_yaw:float = desired_yaw_rate*max_yaw_rate - normalised_gyro_values[GYRO_INDEX_YAW]

            # Proportion calculations
            pid_prop_roll:float = pid_error_roll * pid_kp_roll
            pid_prop_pitch:float = pid_error_pitch * pid_kp_pitch
            pid_prop_yaw:float = pid_error_yaw * pid_kp_yaw

            # Calculate time elapsed since previous PID calculations
            pid_cycle_time:float = time.ticks_diff(time.ticks_us(), prev_pid_timestamp) * 0.000001

            # Integral calculations
            pid_inte_roll:float = pid_error_roll * pid_ki_roll * pid_cycle_time + prev_pid_inte_roll
            pid_inte_pitch:float = pid_error_pitch * pid_ki_pitch * pid_cycle_time + prev_pid_inte_pitch
            pid_inte_yaw:float = pid_error_yaw * pid_ki_yaw * pid_cycle_time + prev_pid_inte_yaw

            # Constrain within integral limits
            pid_inte_roll = max(min(pid_inte_roll, pid_integral_limit_pos), pid_integral_limit_neg)
            pid_inte_pitch = max(min(pid_inte_pitch, pid_integral_limit_pos), pid_integral_limit_neg)
            pid_inte_yaw = max(min(pid_inte_yaw, pid_integral_limit_pos), pid_integral_limit_neg)

            # Derivative calculations
            pid_deri_roll:float = (pid_error_roll - prev_pid_error_roll) * pid_kd_roll * pid_cycle_time
            pid_deri_pitch:float = (pid_error_pitch - prev_pid_error_pitch) * pid_kd_pitch * pid_cycle_time
            pid_deri_yaw:float = (pid_error_yaw - prev_pid_error_yaw) * pid_kd_yaw * pid_cycle_time

            # Capture end timestamp for current PID loop
            prev_pid_timestamp = time.ticks_us()

            throttle_rate:float = THROTTLE_RANGE*desired_throttle_rate + min_throttle_rate
            pid_roll:float = pid_prop_roll + pid_inte_roll + pid_deri_roll
            pid_pitch:float = pid_prop_pitch + pid_inte_pitch + pid_deri_pitch
            pid_yaw:float = pid_prop_yaw + pid_inte_yaw + pid_deri_yaw

            # Throttle calculations (cross configuration)
            motor1_throttle:float = throttle_rate + pid_roll + pid_pitch - pid_yaw
            motor2_throttle:float = throttle_rate - pid_roll + pid_pitch + pid_yaw
            motor3_throttle:float = throttle_rate + pid_roll - pid_pitch + pid_yaw
            motor4_throttle:float = throttle_rate - pid_roll - pid_pitch - pid_yaw

            # Save PID values for subsequent calculations
            prev_pid_error_roll = 0.0
            prev_pid_error_pitch = 0.0
            prev_pid_error_yaw = 0.0
            prev_pid_inte_roll = 0.0
            prev_pid_inte_pitch = 0.0
            prev_pid_inte_yaw = 0.0

            # Calculate duty cycle
            # To be added
        except:
            break

        # print("INFO  >>>>   Loop duration:", pid_cycle_time) # Target duration is less than 0.004 s

    # Out of while loop
else:
    print("ERROR >>>>   Setup failed.\n")

    for error in error_list:
        print("ERROR >>>>  ", error)
