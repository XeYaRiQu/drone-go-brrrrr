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

gyro_range:int = 250 # 250, 500, 1000, 2000 dps (IMU_REG_GYRO_CONFIG[3:4])
acce_range:int = 4   # 2, 4, 8, 16 g (IMU_REG_ACCE_CONFIG[3:4])

min_throttle_rate:float = 0.07  # Maximum throttle at 0% input (does not generate lift)
max_throttle_rate:float = 0.80  # Maximum throttle at 100% input (limits acceleration and throttle)


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
pid_integral_limit_neg:float = -100.0


##### Pin settings #####

LED_GPIO = Pin("LED", Pin.OUT)

RC_GPIO_IN = Pin(5)
RC_GPIO_OUT = Pin(4)

IMU_GPIO_SDA = Pin(12, pull=Pin.PULL_UP)
IMU_GPIO_SCL = Pin(13, pull=Pin.PULL_UP)

MOTOR1_GPIO = Pin(2)
MOTOR2_GPIO = Pin(28)
MOTOR3_GPIO = Pin(16)
MOTOR4_GPIO = Pin(15)


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
GYRO_X_OFFSET = 0
GYRO_Y_OFFSET = 1
GYRO_Z_OFFSET = 2

SCALE_MULTIPLIER_ACCE:float = acce_range / 32767
SCALE_MULTIPLIER_GYRO:float = gyro_range / 32767

THROTTLE_RANGE = max_throttle_rate - min_throttle_rate


##### Enable pin IO #####

rc:UART = UART(1)
imu:I2C = I2C(0, sda=IMU_GPIO_SDA, scl=IMU_GPIO_SCL)
motor1:PWM = PWM(MOTOR1_GPIO)
motor2:PWM = PWM(MOTOR2_GPIO)
motor3:PWM = PWM(MOTOR3_GPIO)
motor4:PWM = PWM(MOTOR4_GPIO)
led=LED_GPIO


##### Others #####

motors_are_armed:bool = False
error_list:list[str] = []
raw_rc_values:list[int] = [0]*6
normalised_rc_values:list[float] = [0.0]*6
normalised_gyro_values:list[float] = [0.0]*3
gyro_offset_bias:list[float] = [0.0]*3

# These are the starting values that will be used in the first PID calculation
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
        imu.writeto_mem(IMU_I2C_ADDRESS, IMU_REG_PWR_MGMT1, b'\x80')   # Reset all registers
        time.sleep_ms(100)
        imu.writeto_mem(IMU_I2C_ADDRESS, IMU_REG_PWR_MGMT1, b'\x09')   # Wake, disable temperature sensor
        time.sleep_ms(100)
        imu.writeto_mem(IMU_I2C_ADDRESS, IMU_REG_SMPLRT_DIV, b'\x03')  # Set sensor sample rate to 250 Hz
        time.sleep_ms(100)
        imu.writeto_mem(IMU_I2C_ADDRESS, IMU_REG_CONFIG, b'\x03')      # Set accelerometer LPF to 44 Hz and gyroscope LPF to 42 Hz
        time.sleep_ms(100)
        imu.writeto_mem(IMU_I2C_ADDRESS, IMU_REG_GYRO_CONFIG, b'\x00') # Set gyroscope scale to 250 dps
        print("INFO  >>>>   MPU-6050 setup --> SUCCESS\n")
    except:
        error_raised_flag = True
        error_list.append("IMU I2C write error.")
        print("ERROR >>>>   MPU-6050 setup --> FAIL\n")

    try:
        # Who am I check
        if int.from_bytes(imu.readfrom_mem(IMU_I2C_ADDRESS, IMU_REG_WHO_AM_I, 1), 'big') == IMU_I2C_ADDRESS:
            print("INFO  >>>>   MPU-6050 verify WHO_AM_I -> SUCCESS\n")
        else:
            error_raised_flag = True
            error_list.append("MPU-6050 WHO_AM_I read error.")
            print("ERROR >>>>   MPU-6050 verify WHO_AM_I -> FAIL\n")

        # Sleep and temperature sensor disabled check
        if int.from_bytes(imu.readfrom_mem(IMU_I2C_ADDRESS, IMU_REG_PWR_MGMT1, 1), 'big') == 9:
            print("INFO  >>>>   MPU-6050 verify IMU_REG_PWR_MGMT1 -> SUCCESS\n")
        else:
            error_raised_flag = True
            error_list.append("MPU-6050 IMU_REG_PWR_MGMT1 not set.")
            print("ERROR >>>>   MPU-6050 verify IMU_REG_PWR_MGMT1 -> FAIL\n")

        # Low pass filter check
        if int.from_bytes(imu.readfrom_mem(IMU_I2C_ADDRESS, IMU_REG_CONFIG, 1), 'big') == 3:
            print("INFO  >>>>   MPU-6050 verify IMU_REG_CONFIG -> SUCCESS\n")
        else:
            error_raised_flag = True
            error_list.append("MPU-6050 IMU_REG_CONFIG not set.")
            print("ERROR >>>>   MPU-6050 verify IMU_REG_CONFIG -> FAIL\n")

        # Sample rate check
        if int.from_bytes(imu.readfrom_mem(IMU_I2C_ADDRESS, IMU_REG_SMPLRT_DIV, 1), 'big') == 3:
            print("INFO  >>>>   MPU-6050 verify IMU_REG_SMPLRT_DIV -> SUCCESS\n")
        else:
            error_raised_flag = True
            error_list.append("MPU-6050 IMU_REG_SMPLRT_DIV not set.")
            print("INFO  >>>>   MPU-6050 verify IMU_REG_SMPLRT_DIV -> FAIL\n")

        # Gyroscope scale check
        if int.from_bytes(imu.readfrom_mem(IMU_I2C_ADDRESS, IMU_REG_GYRO_CONFIG, 1), 'big') == 0:
            print("INFO  >>>>   MPU-6050 verify IMU_REG_GYRO_CONFIG -> SUCCESS\n")
        else:
            error_raised_flag = True
            error_list.append("MPU-6050 IMU_REG_GYRO_CONFIG not set.")
            print("ERROR >>>>   MPU-6050 verify IMU_REG_GYRO_CONFIG -> FAIL\n")
    except:
        error_raised_flag = True
        error_list.append("IMU I2C read error.")
        print("ERROR >>>>   MPU-6050 verify settings -> FAIL\n")

    calc_gyro_bias()

    if not error_raised_flag:
        return 0
    else:
        return 1


def calc_gyro_bias() -> None:
    gyro_bias_x_data:list[float] = []
    gyro_bias_y_data:list[float] = []
    gyro_bias_z_data:list[float] = []
    gyro_bias_data_points:int = 0
    gyro_bias_duration = time.ticks_ms() + 5000
    print("INFO  >>>>   Calculating gyroscope bias. Keep vehicle still.\n")

    while time.ticks_ms() < gyro_bias_duration:
        imu_read()
        gyro_bias_x_data.append(normalised_gyro_values[0])
        gyro_bias_y_data.append(normalised_gyro_values[1])
        gyro_bias_z_data.append(normalised_gyro_values[2])
        gyro_bias_data_points += 1
        time.sleep_ms(50)

    gyro_offset_bias[GYRO_X_OFFSET] = sum(gyro_bias_x_data) / gyro_bias_data_points
    gyro_offset_bias[GYRO_Y_OFFSET] = sum(gyro_bias_y_data) / gyro_bias_data_points
    gyro_offset_bias[GYRO_Z_OFFSET] = sum(gyro_bias_z_data) / gyro_bias_data_points
    print("INFO  >>>>   Gyroscope offsets saved. Offsets:\nINFO  >>>>   X: {}\nINFO  >>>>   Y: {}\nINFO  >>>>   Z: {}\n".format(gyro_offset_bias[GYRO_X_OFFSET], gyro_offset_bias[GYRO_Y_OFFSET], gyro_offset_bias[GYRO_Z_OFFSET]))


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

                    # Normalise data
                    normalised_rc_values[RC_THROTTLE_CH] = float(raw_rc_values[RC_THROTTLE_CH] * 0.001 - 1) # Normalise from 1000-2000 to 0.0-1.0
                    normalised_rc_values[RC_ROLL_CH] = float((raw_rc_values[RC_ROLL_CH] - 1500) * 0.002) # Normalise from 1000-2000 to -1-1
                    normalised_rc_values[RC_PITCH_CH] = float((raw_rc_values[RC_PITCH_CH] - 1500) * 0.002) # Normalise from 1000-2000 to -1-1
                    normalised_rc_values[RC_YAW_CH] = float(-((raw_rc_values[RC_YAW_CH] - 1500) * 0.002)) # Normalise from 1000-2000 to -1-1
                    normalised_rc_values[RC_EXTRA1_CH] = int(raw_rc_values[RC_EXTRA1_CH] * 0.001 - 1) # Normalise from 1000-2000 to 0-1
                    normalised_rc_values[RC_EXTRA2_CH] = int(raw_rc_values[RC_EXTRA2_CH] * 0.001 - 1) # Normalise from 1000-2000 to 0-1

                    break


def imu_read() -> None:
    """
    The IMU measurements are 16-bit 2's complement values ranging from -32768
    to 32767 which needs to be multiplied by the scale multipliers to obtain the
    physical values. These scale multipliers change depending on the range set
    in their respective register configs. The high byte is read before the low
    byte for each measurement.
    """

    x_hi = int.from_bytes(imu.readfrom_mem(IMU_I2C_ADDRESS, IMU_REG_GYRO_X_HI, 1), 'big')
    x_lo = int.from_bytes(imu.readfrom_mem(IMU_I2C_ADDRESS, IMU_REG_GYRO_X_LO, 1), 'big')
    y_hi = int.from_bytes(imu.readfrom_mem(IMU_I2C_ADDRESS, IMU_REG_GYRO_Y_HI, 1), 'big')
    y_lo = int.from_bytes(imu.readfrom_mem(IMU_I2C_ADDRESS, IMU_REG_GYRO_Y_LO, 1), 'big')
    z_hi = int.from_bytes(imu.readfrom_mem(IMU_I2C_ADDRESS, IMU_REG_GYRO_Z_HI, 1), 'big')
    z_lo = int.from_bytes(imu.readfrom_mem(IMU_I2C_ADDRESS, IMU_REG_GYRO_Z_LO, 1), 'big')

    x_value = (x_hi << 8) | x_lo
    y_value = (y_hi << 8) | y_lo
    z_value = (z_hi << 8) | z_lo

    if x_value > 32767:
        normalised_gyro_values[GYRO_INDEX_ROLL] = (x_value - 65536) * SCALE_MULTIPLIER_GYRO - gyro_offset_bias[GYRO_INDEX_ROLL]
    else:
        normalised_gyro_values[GYRO_INDEX_ROLL] = x_value * SCALE_MULTIPLIER_GYRO - gyro_offset_bias[GYRO_INDEX_ROLL]

    if y_value > 32767:
        normalised_gyro_values[GYRO_INDEX_PITCH] = (y_value - 65536) * SCALE_MULTIPLIER_GYRO - gyro_offset_bias[GYRO_INDEX_PITCH]
    else:
        normalised_gyro_values[GYRO_INDEX_PITCH] = y_value * SCALE_MULTIPLIER_GYRO - gyro_offset_bias[GYRO_INDEX_PITCH]

    if z_value > 32767:
        normalised_gyro_values[GYRO_INDEX_YAW] = (z_value - 65536) * SCALE_MULTIPLIER_GYRO - gyro_offset_bias[GYRO_INDEX_YAW]
    else:
        normalised_gyro_values[GYRO_INDEX_YAW] = z_value * SCALE_MULTIPLIER_GYRO - gyro_offset_bias[GYRO_INDEX_YAW]


##### Main #####

if setup() == 0:
    prev_pid_timestamp:int = time.ticks_us()
    print("INFO  >>>>   Starting flight control loop.\n")

    # Flash LED for 5 seconds
    for i in range(5):
        for j in range(5):
            led.toggle()
            time.sleep_ms(200)
    led.off()

    # Main loop start
    while True:
        if motors_are_armed:
            rc_read()

            if normalised_rc_values[RC_EXTRA1_CH] == 0:
                if normalised_rc_values[RC_THROTTLE_CH] == 0.0:
                    motor1.deinit()
                    motor2.deinit()
                    motor3.deinit()
                    motor4.deinit()
                    motors_are_armed = False

            imu_read()
            desired_throttle_rate:float = normalised_rc_values[RC_THROTTLE_CH]
            desired_pitch_rate:float = normalised_rc_values[RC_PITCH_CH]
            desired_roll_rate:float = normalised_rc_values[RC_ROLL_CH]
            desired_yaw_rate:float = normalised_rc_values[RC_YAW_CH]

            # Error calculations (desired - actual)
            pid_error_roll:float = desired_roll_rate * max_roll_rate - normalised_gyro_values[GYRO_INDEX_ROLL]
            pid_error_pitch:float = desired_pitch_rate * max_pitch_rate - normalised_gyro_values[GYRO_INDEX_PITCH]
            pid_error_yaw:float = desired_yaw_rate * max_yaw_rate - normalised_gyro_values[GYRO_INDEX_YAW]

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

            # Calculate time elapsed since previous PID calculations
            # pid_cycle_time:float = time.ticks_diff(time.ticks_us(), prev_pid_timestamp) * 0.000001
            pid_cycle_time:float = 1/time.ticks_diff(time.ticks_us(), prev_pid_timestamp) * 0.000001

            # Derivative calculations
            pid_deri_roll:float = (pid_error_roll - prev_pid_error_roll) * pid_kd_roll * pid_cycle_time
            pid_deri_pitch:float = (pid_error_pitch - prev_pid_error_pitch) * pid_kd_pitch * pid_cycle_time
            pid_deri_yaw:float = (pid_error_yaw - prev_pid_error_yaw) * pid_kd_yaw * pid_cycle_time

            # Capture end timestamp for current PID loop
            prev_pid_timestamp = time.ticks_us()

            throttle_rate:float = desired_throttle_rate * THROTTLE_RANGE + min_throttle_rate
            pid_roll:float = pid_prop_roll + pid_inte_roll + pid_deri_roll
            pid_pitch:float = pid_prop_pitch + pid_inte_pitch + pid_deri_pitch
            pid_yaw:float = pid_prop_yaw + pid_inte_yaw + pid_deri_yaw

            # Throttle calculations (cross configuration)
            motor1_throttle:float = throttle_rate - pid_roll + pid_pitch + pid_yaw
            motor2_throttle:float = throttle_rate + pid_roll + pid_pitch - pid_yaw
            motor3_throttle:float = throttle_rate + pid_roll - pid_pitch + pid_yaw
            motor4_throttle:float = throttle_rate - pid_roll - pid_pitch - pid_yaw

            # Save PID values for subsequent calculations
            prev_pid_error_roll = pid_error_roll
            prev_pid_error_pitch = pid_error_pitch
            prev_pid_error_yaw = pid_error_yaw
            prev_pid_inte_roll = pid_inte_roll
            prev_pid_inte_pitch = pid_inte_pitch
            prev_pid_inte_yaw = pid_inte_yaw

            # Calculate duty cycle
            motor1.duty_ns(int(min(max(motor1_throttle * 1000000, 0) + 1000000, 2000000)))
            motor2.duty_ns(int(min(max(motor2_throttle * 1000000, 0) + 1000000, 2000000)))
            motor3.duty_ns(int(min(max(motor3_throttle * 1000000, 0) + 1000000, 2000000)))
            motor4.duty_ns(int(min(max(motor4_throttle * 1000000, 0) + 1000000, 2000000)))

            # DEBUG PRINTS
            # print(normalised_rc_values)
            # print(normalised_gyro_values)
            print(motor1_throttle, motor2_throttle, motor3_throttle, motor4_throttle)
            # print("INFO  >>>>   Loop duration:", pid_cycle_time) # Target duration is less than 0.004 s
        else:
            rc_read()
            if normalised_rc_values[RC_EXTRA1_CH] == 1:
                if normalised_rc_values[RC_THROTTLE_CH] == 0.0:
                    motor1.freq(250)
                    motor2.freq(250)
                    motor3.freq(250)
                    motor4.freq(250)
                    motors_are_armed = True
                    takeoff_delay = time.ticks_ms() + 1000

                    while time.ticks_ms() < takeoff_delay:
                        motor1.duty_ns(1000000)
                        motor2.duty_ns(1000000)
                        motor3.duty_ns(1000000)
                        motor4.duty_ns(1000000)
            else:
                    motor1.deinit()
                    motor2.deinit()
                    motor3.deinit()
                    motor4.deinit()
    # Main loop end
else:
    print("ERROR >>>>   Setup failed.\n")

    for error in error_list:
        print("ERROR >>>>  ", error)

    while True:
        led.toggle()
        time.sleep_ms(50)
