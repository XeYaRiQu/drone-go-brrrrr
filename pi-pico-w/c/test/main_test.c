/*
 * Only change values in Settings.
*/

////////////////// Includes //////////////////

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <fastmath.h>
#include <float.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/cyw43_arch.h" // WiFi chip on Pico W
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "hardware/uart.h"
#include "hardware/clocks.h"


////////////////// Settings //////////////////

#define MIN_THROTTLE   0.07f
#define MAX_THROTTLE   0.90f
#define THROTTLE_RANGE (MAX_THROTTLE - MIN_THROTTLE)
#define MAX_YAW_RATE   30.0
#define MAX_ROLL_RATE  30.0
#define MAX_PITCH_RATE 30.0

/* Pin placement */
#define PIN_LED     0  // The LED pin is tied to CYW43
#define PIN_RC_RX   5  // UART1
#define PIN_RC_TX   4  // UART1
#define PIN_IMU_SDA 12 // I2C0
#define PIN_IMU_SCL 13 // I2C0
#define PIN_MOTOR1  16 // PWM channel: 0A
#define PIN_MOTOR2  15 // PWM channel: 7B
#define PIN_MOTOR3  2  // PWM channel: 1A
#define PIN_MOTOR4  28 // PWM channel: 6A

/* IMU settings */
enum GYRO_RANGE {
    RANGE_250DPS = 0,
    RANGE_500DPS = 4,
    RANGE_1000DPS = 8,
    RANGE_2000DPS = 12
};

enum ACCEL_RANGE {
    RANGE_2G = 0,
    RANGE_4G = 4,
    RANGE_8G = 8,
    RANGE_16G = 16
};

enum DLPF_BANDWIDTH {
    GYRO_256Hz = 0, // 0.98 ms delay
    GYRO_188Hz,     // 1.9 ms delay
    GYRO_98Hz,      // 2.8 ms delay
    GYRO_42Hz,      // 4.8 ms delay
    GYRO_20Hz,      // 8.3 ms delay
    GYRO_10Hz,      // 13.4 ms delay
    GYRO_5Hz,       // 18.6 ms delay
};

int GYRO_RANGE = RANGE_1000DPS;
int ACCEL_RANGE = RANGE_4G;
int LPF_CONFIG_BYTE = GYRO_98Hz;

/* PID values */
static const double KP_ROLL = 0.00043714285;
static const double KP_PITCH = 0.00043714285;
static const double KP_YAW = 0.001714287;

static const double KI_ROLL = 0.00255;
static const double KI_PITCH = 0.00255;
static const double KI_YAW = 0.003428571;

static const double KD_ROLL = 0.00002571429;
static const double KD_PITCH = 0.00002571429;
static const double KD_YAW = 0.0;

static const float I_LIMIT_POS = 100.0f;
static const float I_LIMIT_NEG = -100.0f;


////////////////// Constants //////////////////

#define IMU_I2C_ADDRESS  104
#define IMU_SMPLRT_DIV   25
#define IMU_PWR_MGMT1    107
#define IMU_WHO_AM_I     117
#define IMU_CONFIG       26
#define IMU_GYRO_CONFIG  27
#define IMU_ACCEL_CONFIG 28
#define IMU_SELF_TEST_X  13
#define IMU_SELF_TEST_Y  14
#define IMU_SELF_TEST_Z  15
#define IMU_SELF_TEST_A  16
#define IMU_ACCE_X_H     59
#define IMU_ACCE_X_L     60
#define IMU_ACCE_Y_H     61
#define IMU_ACCE_Y_L     62
#define IMU_ACCE_Z_H     3
#define IMU_ACCE_Z_L     64
#define IMU_TEMP_HI      65
#define IMU_TEMP_LO      66
#define IMU_GYRO_X_H     67
#define IMU_GYRO_X_L     68
#define IMU_GYRO_Y_H     69
#define IMU_GYRO_Y_L     70
#define IMU_GYRO_Z_H     71
#define IMU_GYRO_Z_L     72

#define RESET_ALL_BYTE         0b10000000
#define WAKE_TEMP_DISABLE_BYTE 0b00001001
#define WAKE_TEMP_ENABLE_BYTE  0b00000001
#define SAMPLE_RATE_BYTE       0b00000000

#define ACCEL_X 0
#define ACCEL_Y 1
#define ACCEL_Z 2

#define GYRO_ROLL  0
#define GYRO_PITCH 1
#define GYRO_YAW   2

/* Channel order in IBUS packet */
#define RC_THROTTLE 2
#define RC_ROLL     0
#define RC_PITCH    1
#define RC_YAW      3
#define RC_SWA      4
#define RC_SWD      5

#define F_SYS 250000000
#define F_PWM 250


////////////////// Global variables //////////////////

float gyro_multiplier, accel_multiplier;
int gyro_config_byte, accel_config_byte;

float normalised_rc_values[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float normalised_gyro_values[3], normalised_accel_values[3];

float gyro_x_bias = 0.0f;
float gyro_y_bias = 0.0f; 
float gyro_z_bias = 0.0f;
float accel_x_bias = 0.0f;
float accel_y_bias = 0.0f;
float accel_z_bias = 0.0f;

uint16_t motor1_pwm_level, motor2_pwm_level, motor3_pwm_level, motor4_pwm_level, wrap_num, esc_max;


////////////////// Functions //////////////////

void write_register(i2c_inst_t *i2c, int addr, int reg, int val) {
    uint8_t buffer[] = {(uint8_t) reg, (uint8_t) val};
    i2c_write_blocking(i2c, addr, buffer, 2, false);
}


int read_register(i2c_inst_t *i2c, int addr, int reg) {
    uint8_t val = reg;
    uint8_t readout;
    i2c_write_blocking(i2c, addr, &val, 1, true);
    i2c_read_blocking(i2c, addr, &readout, 1, false);
    return readout;
}


int mpu6050_init() {
    int fail_flag = 0;

    switch (GYRO_RANGE) {
        case RANGE_250DPS:
            gyro_multiplier = 250.0f/32768.0f;
            gyro_config_byte = 0b00000000; // 0x00, 0
            break;
        case RANGE_500DPS:
            gyro_multiplier = 500.0f/32768.0f;
            gyro_config_byte = 0b00001000; // 0x08, 8
            break;
        case RANGE_1000DPS:
            gyro_multiplier = 1000.0f/32768.0f;
            gyro_config_byte = 0b00010000; // 0x10, 16
            break;
        case RANGE_2000DPS:
            gyro_multiplier = 2000.0f/32768.0f;
            gyro_config_byte = 0b00011000; // 0x18, 24
            break;
    }

    switch (ACCEL_RANGE) {
        case RANGE_2G:
            accel_multiplier = 2.0f/32768.0f;
            accel_config_byte = 0b00000000; // 0x00, 0
            break;
        case RANGE_4G:
            accel_multiplier = 4.0f/32768.0f;
            accel_config_byte = 0b00001000; // 0x08, 8
            break;
        case RANGE_8G:
            accel_multiplier = 8.0f/32768.0f;
            accel_config_byte = 0b00010000; // 0x10, 16
            break;
        case RANGE_16G:
            accel_multiplier = 16.0f/32768.0f;
            accel_config_byte = 0b00011000; // 0x18, 24
            break;
    }

    // Reset all registers
    write_register(i2c0, IMU_I2C_ADDRESS, IMU_PWR_MGMT1, RESET_ALL_BYTE);
    sleep_ms(10);

    // Wake, disable temperature sensor
    write_register(i2c0, IMU_I2C_ADDRESS, IMU_PWR_MGMT1, WAKE_TEMP_DISABLE_BYTE);
    sleep_ms(10);

    // Set sensor sample rate to 250 Hz (Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV))
    write_register(i2c0, IMU_I2C_ADDRESS, IMU_SMPLRT_DIV, SAMPLE_RATE_BYTE);
    sleep_ms(10);

    // Set DLPF
    write_register(i2c0, IMU_I2C_ADDRESS, IMU_CONFIG, LPF_CONFIG_BYTE);
    sleep_ms(10);

    // Set gyroscope scale (in dps)
    write_register(i2c0, IMU_I2C_ADDRESS, IMU_GYRO_CONFIG, gyro_config_byte);
    sleep_ms(10);

    // Set accelerometer scale (in G)
    write_register(i2c0, IMU_I2C_ADDRESS, IMU_ACCEL_CONFIG, accel_config_byte);
    sleep_ms(10);

    // Verify registers
    if (read_register(i2c0, IMU_I2C_ADDRESS, IMU_WHO_AM_I) != IMU_I2C_ADDRESS) {
        fail_flag = 1;
        printf("ERROR >>>>   Verify MPU-6050 register: WHO_AM_I --> FAIL\n\n");
    }

    if (read_register(i2c0, IMU_I2C_ADDRESS, IMU_PWR_MGMT1) != WAKE_TEMP_DISABLE_BYTE) {
        fail_flag = 1;
        printf("ERROR >>>>   Verify MPU-6050 register: PWR_MGMT_1 --> FAIL\n\n");
    }

    if (read_register(i2c0, IMU_I2C_ADDRESS, IMU_SMPLRT_DIV) != SAMPLE_RATE_BYTE) {
        fail_flag = 1;
        printf("ERROR >>>>   Verify MPU-6050 register: SMPLRT_DIV --> FAIL\n\n");
    }

    if (read_register(i2c0, IMU_I2C_ADDRESS, IMU_CONFIG) != LPF_CONFIG_BYTE) {
        fail_flag = 1;
        printf("ERROR >>>>   Verify MPU-6050 register: CONFIG --> FAIL\n\n");
    }

    if (read_register(i2c0, IMU_I2C_ADDRESS, IMU_GYRO_CONFIG) != gyro_config_byte) {
        fail_flag = 1;
        printf("ERROR >>>>   Verify MPU-6050 register: GYRO_CONFIG --> FAIL\n\n");
    }

    if (read_register(i2c0, IMU_I2C_ADDRESS, IMU_ACCEL_CONFIG) != accel_config_byte) {
        fail_flag = 1;
        printf("ERROR >>>>   Verify MPU-6050 register: ACCEL_CONFIG --> FAIL\n\n");
    }

    return fail_flag;
};


int mpu_6050_self_test() {
    /*
    The self-test response (STR) is defined as follows:

            STR = sensor_output_with_ST_enabled - sensor_output_with_ST_disabled

    The maximum allowable deviation of the sensors from factory trim (FT) is ±14%
    The change from FT of the STR is defined as follows:

            Δ = (STR - FT) / FT

    Before performing self-test, the accelerometer should be set to ±8 G and gyroscope ±250 dps.
    */
    int fail_flag = 0;
    uint8_t raw_data[4];

    // Configure sensors for self-test
    write_register(i2c0, IMU_I2C_ADDRESS, IMU_ACCEL_CONFIG, 0b11110000);
    sleep_ms(10);
    write_register(i2c0, IMU_I2C_ADDRESS, IMU_GYRO_CONFIG, 0b11100000);
    sleep_ms(250); // Delay for self-test to complete

    // Grab self-test results
    raw_data[0] = read_register(i2c0, IMU_I2C_ADDRESS, IMU_SELF_TEST_X);
    raw_data[1] = read_register(i2c0, IMU_I2C_ADDRESS, IMU_SELF_TEST_Y);
    raw_data[2] = read_register(i2c0, IMU_I2C_ADDRESS, IMU_SELF_TEST_Z);
    raw_data[3] = read_register(i2c0, IMU_I2C_ADDRESS, IMU_SELF_TEST_A);

    // Extract accelerometer test results
    uint8_t xa_test = ((raw_data[0] & 0b11100000) >> 3) | ((raw_data[3] & 0b00110000) >> 4);
    uint8_t ya_test = ((raw_data[1] & 0b11100000) >> 3) | ((raw_data[3] & 0b00001100) >> 2);
    uint8_t za_test = ((raw_data[2] & 0b11100000) >> 3) | ((raw_data[3] & 0b00000011));

    printf("INFO  >>>>   ACCELEROMETER SELF-TEST VALUES:\n");
    printf("INFO  >>>>   X: %d   Y: %d   Z: %d\n", xa_test, ya_test, za_test);

    // Extract gyroscope test results
    uint8_t xg_test = raw_data[0] & 0b00011111;
    uint8_t yg_test = raw_data[1] & 0b00011111;
    uint8_t zg_test = raw_data[2] & 0b00011111;

    printf("INFO  >>>>   GYROSCOPE SELF-TEST VALUES:\n");
    printf("INFO  >>>>   X: %d   Y: %d   Z: %d\n\n", xg_test, yg_test, zg_test);

    // Obtain factory-trim values
    float xa_ft = 1392.64f * pow(0.92f / 0.34f, ((float) (xa_test - 1)) / 30);
    float ya_ft = 1392.64f * pow(0.92f / 0.34f, ((float) (ya_test - 1)) / 30);
    float za_ft = 1392.64f * pow(0.92f / 0.34f, ((float) (za_test - 1)) / 30);
    float xg_ft = 3275.0f * pow(1.046f, (float) (xg_test - 1));
    float yg_ft = -3275.0f * pow(1.046f, (float) (yg_test - 1));
    float zg_ft = 3275.0f * pow(1.046f, (float) (zg_test - 1));

    printf("INFO  >>>>   ACCELEROMETER FACTORY TRIM VALUES:\n");
    printf("INFO  >>>>   X: %f   Y: %f   Z: %f\n", xa_ft, ya_ft, za_ft);
    printf("INFO  >>>>   GYROSCOPE FACTORY TRIM VALUES:\n");
    printf("INFO  >>>>   X: %f   Y: %f   Z: %f\n\n", xg_ft, yg_ft, zg_ft);

    float xa_change = (xa_test - xa_ft) / xa_ft;
    float ya_change = (ya_test - ya_ft) / ya_ft;
    float za_change = (za_test - za_ft) / za_ft;
    float xg_change = (xg_test - xg_ft) / xg_ft;
    float yg_change = (yg_test - yg_ft) / yg_ft;
    float zg_change = (zg_test - zg_ft) / zg_ft;

    if (xa_change > 0.14f || xa_change < -0.14f) {
        fail_flag = 1;
        printf("ERROR >>>>   Deviation from FT_XA: %f\n", xa_change);
    }

    if (ya_change > 0.14f || ya_change < -0.14f) {
        fail_flag = 1;
        printf("ERROR >>>>   Deviation from FT_YA: %f\n", ya_change);
    }

    if (za_change > 0.14f || za_change < -0.14f) {
        fail_flag = 1;
        printf("ERROR >>>>   Deviation from FT_ZA: %f\n", za_change);
    }

    if (xg_change > 0.14f || xg_change < -0.14f) {
        fail_flag = 1;
        printf("ERROR >>>>   Deviation from FT_XG: %f\n", xg_change);
    }

    if (yg_change > 0.14f || yg_change < -0.14f) {
        fail_flag = 1;
        printf("ERROR >>>>   Deviation from FT_YG: %f\n", yg_change);
    }

    if (zg_change > 0.14f || zg_change < -0.14f) {
        fail_flag = 1;
        printf("ERROR >>>>   Deviation from FT_ZG: %f\n", zg_change);
    }

    return fail_flag;
}


void rc_read() {
    /*
    An iBus packet comprises 32 bytes:
        - 2 header bytes (first header is 0x20, second is 0x40)
        - 28 channel bytes (2 bytes per channel value)
        - 2 checksum bytes (1 checksum value)

    The protocol data rate is 115200 baud. A packet is transmitted every 7 ms.
    Each packet takes 32*8/115200 seconds to transmit (about 2.22 ms).
    Checksum = 0xFFFF - sum of first 30 bytes
    The channels' values are transmitted sequentially in little endian byte order.
    */
    for (int attempt = 0; attempt < 6; ++attempt) {
        // Check if data is available in the UART buffer
        if (uart_is_readable(uart1)) {
            // Read start bytes
            uint8_t char1 = uart_getc(uart1);

            if (char1 == 0) {
                break; // Prevents exception when only 1 char is avail in UART1
            }

            uint8_t char2 = uart_getc(uart1);

            // Validate start bytes
            if (char1 == 0x20 && char2 == 0x40) {
                uint8_t buffer[30];

                // Read the rest of the data into the buffer
                uart_read_blocking(uart1, buffer, 30);

                uint16_t checksum = 0xFF9F; // 0xFFFF - 0x20 - 0x40

                for (int byte_index = 0; byte_index < 28; ++byte_index) {
                    checksum -= buffer[byte_index];
                }

                // Validate checksum
                if (checksum == ((buffer[29] << 8) | buffer[28])) { // Convert to big endian
                    // Process raw RC values
                    int raw_rc_values[6];

                    for (int channel = 0; channel < 6; ++channel) {
                        raw_rc_values[channel] = (buffer[channel * 2 + 1] << 8) + buffer[channel * 2];
                    }

                    // Normalize data
                    normalised_rc_values[RC_THROTTLE] = raw_rc_values[RC_THROTTLE] * 0.001f - 1.0f; // Normalize from 1000-2000 to 0.0-1.0
                    normalised_rc_values[RC_ROLL] = (raw_rc_values[RC_ROLL] - 1500) * 0.002f;       // Normalize from 1000-2000 to -1-1
                    normalised_rc_values[RC_PITCH] = (raw_rc_values[RC_PITCH] - 1500) * 0.002f;     // Normalize from 1000-2000 to -1-1
                    normalised_rc_values[RC_YAW] = -((raw_rc_values[RC_YAW] - 1500) * 0.002f);      // Normalize from 1000-2000 to -1-1
                    normalised_rc_values[RC_SWA] = raw_rc_values[RC_SWA] * 0.001f - 1.0f;           // Normalize from 1000-2000 to 0-1
                    normalised_rc_values[RC_SWD] = raw_rc_values[RC_SWD] * 0.001f - 1.0f;           // Normalize from 1000-2000 to 0-1

                    break;
                }
            }
        }
    }
}


void imu_read() {
    /*
    The IMU measurements are 16-bit 2's complement values ranging from -32768
    to 32767 which needs to be multiplied by the scale multipliers to obtain the
    physical values. These scale multipliers change depending on the range set
    in their respective register configs. The high byte is read before the low
    byte for each axis measurement which is then combined to form the 16-bit values.
    */
    uint8_t accel_buffer[6], gyro_buffer[6];
    uint16_t raw_accel_data[3], raw_gyro_data[3];
    uint8_t accel_xout_h = IMU_ACCE_X_H;
    uint8_t gyro_xout_h = IMU_GYRO_X_H;

    // Get raw accelerometer data
    i2c_write_blocking(i2c_default, IMU_I2C_ADDRESS, &accel_xout_h, 1, true);
    i2c_read_blocking(i2c_default, IMU_I2C_ADDRESS, accel_buffer, 6, false);

    // Get raw gyro data
    i2c_write_blocking(i2c_default, IMU_I2C_ADDRESS, &gyro_xout_h, 1, true);
    i2c_read_blocking(i2c_default, IMU_I2C_ADDRESS, gyro_buffer, 6, false);

    for (int i = 0; i < 3; ++i) {
        raw_accel_data[i] = (accel_buffer[i * 2] << 8 | accel_buffer[(i * 2) + 1]);
        raw_gyro_data[i] = (gyro_buffer[i * 2] << 8 | gyro_buffer[(i * 2) + 1]);
    }

    // Physical orientation of the IMU is North-West-Up for X-Y-Z. Take negative values on pitch and yaw to convert it to North-East-Down
    normalised_accel_values[ACCEL_X] = (raw_accel_data[ACCEL_X] > 32767) ? ((raw_accel_data[ACCEL_X] - 65536) * accel_multiplier - accel_x_bias) : (raw_accel_data[ACCEL_X] * accel_multiplier - accel_x_bias);
    normalised_accel_values[ACCEL_Y] = (raw_accel_data[ACCEL_Y] > 32767) ? ((raw_accel_data[ACCEL_Y] - 65536) * accel_multiplier - accel_y_bias) : (raw_accel_data[ACCEL_Y] * accel_multiplier - accel_y_bias);
    normalised_accel_values[ACCEL_Z] = (raw_accel_data[ACCEL_Z] > 32767) ? ((raw_accel_data[ACCEL_Z] - 65536) * accel_multiplier - accel_z_bias) : (raw_accel_data[ACCEL_Z] * accel_multiplier - accel_z_bias);
    normalised_gyro_values[GYRO_ROLL] = (raw_gyro_data[GYRO_ROLL] > 32767) ? ((raw_gyro_data[GYRO_ROLL] - 65536) * gyro_multiplier - gyro_x_bias) : (raw_gyro_data[GYRO_ROLL] * gyro_multiplier - gyro_x_bias);
    normalised_gyro_values[GYRO_PITCH] = (raw_gyro_data[GYRO_PITCH] > 32767) ? -((raw_gyro_data[GYRO_PITCH] - 65536) * gyro_multiplier + gyro_y_bias) : -(raw_gyro_data[GYRO_PITCH] * gyro_multiplier + gyro_y_bias);
    normalised_gyro_values[GYRO_YAW] = (raw_gyro_data[GYRO_YAW] > 32767) ? -((raw_gyro_data[GYRO_YAW] - 65536) * gyro_multiplier + gyro_z_bias) : (raw_gyro_data[GYRO_YAW] * gyro_multiplier - gyro_z_bias);
}


void mpu_6050_cali() {
    float gyro_x_sum = 0.0f;
    float gyro_y_sum = 0.0f; 
    float gyro_z_sum = 0.0f;
    float accel_x_sum = 0.0f;
    float accel_y_sum = 0.0f;
    float accel_z_sum = 0.0f;
    int data_points = 0;
    uint64_t calibration_time = time_us_64() + 3000000;

    while (time_us_64() < calibration_time) {
        imu_read();

        accel_x_sum += normalised_accel_values[0];
        accel_y_sum += normalised_accel_values[1];
        accel_z_sum += normalised_accel_values[2];
        gyro_x_sum += normalised_gyro_values[0];
        gyro_y_sum += normalised_gyro_values[1];
        gyro_z_sum += normalised_gyro_values[2];
        // printf("X: %f    Y: %f    Z: %f\n", normalised_accel_values[0], normalised_accel_values[1], normalised_accel_values[2]);
        // printf("X: %f    Y: %f    Z: %f\n", normalised_gyro_values[0], normalised_gyro_values[1], normalised_gyro_values[2]);
        data_points = ++data_points;
        sleep_ms(3);
    }

    // Average bias offsets
    printf("INFO  >>>>   %d data points collected. Averaging bias values\n\n", data_points);
    accel_x_bias = accel_x_sum / data_points;
    accel_y_bias = accel_y_sum / data_points;
    accel_z_bias = accel_z_sum / data_points;
    gyro_x_bias = gyro_x_sum / data_points;
    gyro_y_bias = gyro_y_sum / data_points;
    gyro_z_bias = gyro_z_sum / data_points;

    printf("INFO  >>>>   ACCELEROMETER OFFSETS (G)\n");
    printf("INFO  >>>>   X: %f    Y: %f    Z: %f\n", accel_x_bias, accel_y_bias, accel_z_bias);
    printf("INFO  >>>>   GYROSCOPE OFFSETS (DPS)\n");
    printf("INFO  >>>>   X: %f    Y: %f    Z: %f\n\n", gyro_x_bias, gyro_y_bias, gyro_z_bias);
}


void motor_pwm_init() {
    /*
    The Pico has 8 PWM slices with each slice having 2 PWM channels.

                  (slice)                  ┌-->  Channel A
    sys_clk  -->  clk_div  -->  counter  --|
                                           └-->  Channel B

    The counter and clock divider are 16-bit. The sys_clk is set to 250 MHz during setup.
    To maximise resolution of duty cycles that can be set, the wrap number should be as
    high as possible (i.e. close to 65535).
    */
    float clk_div = (((float) F_SYS / (4096 * F_PWM) + 1)) / 16;
    wrap_num = F_SYS / (clk_div * F_PWM) - 1;
    esc_max = wrap_num/2;
    int motor1_pwm_slice = pwm_gpio_to_slice_num(PIN_MOTOR1);
    int motor2_pwm_slice = pwm_gpio_to_slice_num(PIN_MOTOR2);
    int motor3_pwm_slice = pwm_gpio_to_slice_num(PIN_MOTOR3);
    int motor4_pwm_slice = pwm_gpio_to_slice_num(PIN_MOTOR4);

    printf("INFO  >>>>   Clock divider set to: %f\n", clk_div);
    printf("INFO  >>>>   Wrap number set to: %d\n", wrap_num);
    printf("INFO  >>>>   Max PWM level: %d\n\n", esc_max);

    gpio_set_function(PIN_MOTOR1, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR2, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR3, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR4, GPIO_FUNC_PWM);

    // Reset PWM
    pwm_set_mask_enabled(0);
    pwm_set_counter(motor1_pwm_slice, 0);
    pwm_set_counter(motor2_pwm_slice, 0);
    pwm_set_counter(motor3_pwm_slice, 0);
    pwm_set_counter(motor4_pwm_slice, 0);

    // Re-enable PWM
    pwm_set_enabled(motor1_pwm_slice, true);
    pwm_set_enabled(motor2_pwm_slice, true);
    pwm_set_enabled(motor3_pwm_slice, true);
    pwm_set_enabled(motor4_pwm_slice, true);

    // Set the counter frequency (f_counter = f_sys / clk_div)
    pwm_set_clkdiv(motor1_pwm_slice, clk_div);
    pwm_set_clkdiv(motor2_pwm_slice, clk_div);
    pwm_set_clkdiv(motor3_pwm_slice, clk_div);
    pwm_set_clkdiv(motor4_pwm_slice, clk_div);

    // Wrap number sets the frequency of the PWM signal (wrap_number = f_counter / f_pwm - 1)
    pwm_set_wrap(motor1_pwm_slice, wrap_num);
    pwm_set_wrap(motor2_pwm_slice, wrap_num);
    pwm_set_wrap(motor3_pwm_slice, wrap_num);
    pwm_set_wrap(motor4_pwm_slice, wrap_num);

    // Level sets the duty cycle (level = wrap_number * duty_cycle)
    pwm_set_gpio_level(PIN_MOTOR1, 0);
    pwm_set_gpio_level(PIN_MOTOR2, 0);
    pwm_set_gpio_level(PIN_MOTOR3, 0);
    pwm_set_gpio_level(PIN_MOTOR4, 0);
}


////////////////// Setup //////////////////

int setup() {
    int fail_flag = 0;
    uint64_t setup_delay = time_us_64() + 5000000;
    stdio_init_all();

    while (time_us_64() < setup_delay); // Delay for serial monitor to establish connection

    // Overclock RP2040
    set_sys_clock_khz(250000, true);
    if (clock_get_hz(clk_sys) == 250000000) {
        printf("INFO  >>>>   Overclock RP2040 to 250 MHz --> SUCCESS\n\n");
    }
    else {
        fail_flag = 1;
        printf("ERROR >>>>   Overclock RP2040 to 250 MHz --> FAIL\n\n");
    }

    // Initialise WiFi chip for LED
    if (cyw43_arch_init() == 0) {
        printf("INFO  >>>>   Initialise WiFi chip --> SUCCESS\n\n");
    }
    else {
        fail_flag = 1;
        printf("ERROR >>>>   Initialise WiFi chip --> FAIL\n\n");
    }

    // Initialise UART for RC
    int baud = uart_init(uart1, 115200);
    if (baud > 115100 && baud < 115300) {
        gpio_set_function(PIN_RC_RX, GPIO_FUNC_UART);
        gpio_set_function(PIN_RC_TX, GPIO_FUNC_UART);
        printf("INFO  >>>>   Initialise UART --> SUCCESS\n\n");
    }
    else {
        fail_flag = 1;
        printf("ERROR >>>>   Initialise UART --> FAIL\n\n");
    }

    // Initialise I2C for IMU
    if (i2c_init(i2c0, 400000) == 400000) {
        gpio_set_function(PIN_IMU_SCL, GPIO_FUNC_I2C);
        gpio_set_function(PIN_IMU_SDA, GPIO_FUNC_I2C);
        gpio_pull_up(PIN_IMU_SCL);
        gpio_pull_up(PIN_IMU_SDA);
        // Make the I2C pins available to picotool (copied from mpu6050_i2c.c in pico-examples)
        bi_decl(bi_2pins_with_func(PIN_IMU_SDA, PIN_IMU_SCL, GPIO_FUNC_I2C));
        printf("INFO  >>>>   Initialise I2C --> SUCCESS\n\n");
    }
    else {
        fail_flag = 1;
        printf("INFO  >>>>   Initialise I2C --> FAIL\n\n");
    }

    // Initialise IMU
    if (mpu6050_init() == 0) {
        printf("INFO  >>>>   Initialise MPU-6050 --> SUCCESS\n\n");
    }
    else {
        fail_flag = 1;
        printf("ERROR >>>>   Initialise MPU-6050 --> FAIL\n\n");
    }

    // Perform IMU self-test
    // if (mpu_6050_self_test() == 0) {
    //     printf("INFO  >>>>   Self-test MPU-6050 --> SUCCESS\n\n");
    // }
    // else {
    //     fail_flag = 1;
    //     printf("ERROR >>>>   Self-test MPU-6050 --> FAIL\n\n");
    // }

    if (fail_flag == 0) {
        // Calibrate IMU
        printf("INFO  >>>>   Calibrating MPU-6050\n\n");
        mpu_6050_cali();

        // Configure PWM generators for ESC output
        printf("INFO  >>>>   Configuring PWM generators\n\n");
        motor_pwm_init();
    }

    return fail_flag;
}


////////////////// Main //////////////////

void main() {
    bool motors_are_armed = false;
    uint64_t start_timestamp = time_us_64();

    printf("INFO  >>>>   Executing setup sequence.\n\n");
    if (setup() == 0) {
        float pid_error_roll = 0.0f;
        float pid_error_pitch = 0.0f;
        float pid_error_yaw = 0.0f;
        float prev_error_roll = 0.0f;
        float prev_error_pitch = 0.0f;
        float prev_error_yaw = 0.0f;
        float prev_integ_roll = 0.0f;
        float prev_integ_pitch = 0.0f;
        float prev_integ_yaw = 0.0f;
        uint64_t loop_end_time;

        printf("INFO  >>>>   Setup completed in %f seconds, looping.\n\n", ((time_us_64() - start_timestamp) * 0.000001f - 5.0f));
        /* Only uncomment this when storing in flash */
        // cyw43_arch_gpio_put(PIN_LED, 1);

        ////////////////// Loop //////////////////
        while (true) {
            loop_end_time = time_us_64() + 4000;

            // Emergency killswitch
            if (normalised_rc_values[RC_SWD] > 0.5f){
                pwm_set_gpio_level(PIN_MOTOR1, 0);
                pwm_set_gpio_level(PIN_MOTOR2, 0);
                pwm_set_gpio_level(PIN_MOTOR3, 0);
                pwm_set_gpio_level(PIN_MOTOR4, 0);
                pwm_set_mask_enabled(0);

                break;
            }

            if (motors_are_armed) {
                rc_read();

                // Disarm motors at low throttle
                if (normalised_rc_values[RC_SWA] < 0.5f) {
                    if (normalised_rc_values[RC_THROTTLE] < 0.05f){
                        pwm_set_gpio_level(PIN_MOTOR1, 0);
                        pwm_set_gpio_level(PIN_MOTOR2, 0);
                        pwm_set_gpio_level(PIN_MOTOR3, 0);
                        pwm_set_gpio_level(PIN_MOTOR4, 0);
                        motors_are_armed = false;
                    }
                }

                imu_read();

                float desired_throttle = normalised_rc_values[RC_THROTTLE];
                float desired_pitch_rate = normalised_rc_values[RC_PITCH];
                float desired_roll_rate = normalised_rc_values[RC_ROLL];
                float desired_yaw_rate = normalised_rc_values[RC_YAW];

                // Error calculations (desired - actual)
                pid_error_roll = desired_roll_rate * MAX_ROLL_RATE - normalised_gyro_values[GYRO_ROLL];
                pid_error_pitch = desired_pitch_rate * MAX_PITCH_RATE - normalised_gyro_values[GYRO_PITCH];
                pid_error_yaw = desired_yaw_rate * MAX_YAW_RATE - normalised_gyro_values[GYRO_YAW];

                // Proportion calculations
                float pid_prop_roll = pid_error_roll * KP_ROLL;
                float pid_prop_pitch = pid_error_pitch * KP_PITCH;
                float pid_prop_yaw = pid_error_yaw * KP_YAW;

                // Integral calculations (multiply by dt)
                float pid_inte_roll = pid_error_roll * KI_ROLL + prev_integ_roll;
                float pid_inte_pitch = pid_error_pitch * KI_PITCH + prev_integ_pitch;
                float pid_inte_yaw = pid_error_yaw * KI_YAW + prev_integ_yaw;

                // Enforce integral limits
                pid_inte_roll = (pid_inte_roll > I_LIMIT_POS) ? I_LIMIT_POS : ((pid_inte_roll < I_LIMIT_NEG) ? I_LIMIT_NEG : pid_inte_roll);
                pid_inte_pitch = (pid_inte_pitch > I_LIMIT_POS) ? I_LIMIT_POS : ((pid_inte_pitch < I_LIMIT_NEG) ? I_LIMIT_NEG : pid_inte_pitch);
                pid_inte_yaw = (pid_inte_yaw > I_LIMIT_POS) ? I_LIMIT_POS : ((pid_inte_yaw < I_LIMIT_NEG) ? I_LIMIT_NEG : pid_inte_yaw);

                // Derivative calculations (divide by dt)
                float pid_deri_roll = (pid_error_roll - prev_error_roll) * KD_ROLL * 250;
                float pid_deri_pitch = (pid_error_pitch - prev_error_pitch) * KD_PITCH * 250;
                float pid_deri_yaw = (pid_error_yaw - prev_error_yaw) * KD_YAW * 250;

                float throttle = desired_throttle * THROTTLE_RANGE + MIN_THROTTLE;
                float pid_roll = pid_prop_roll + pid_inte_roll + pid_deri_roll;
                float pid_pitch = pid_prop_pitch + pid_inte_pitch + pid_deri_pitch;
                float pid_yaw = pid_prop_yaw + pid_inte_yaw + pid_deri_yaw;

                // Save PID values for subsequent calculations
                prev_error_roll = pid_error_roll;
                prev_error_pitch = pid_error_pitch;
                prev_error_yaw = pid_error_yaw;
                prev_integ_roll = pid_inte_roll;
                prev_integ_pitch = pid_inte_pitch;
                prev_integ_yaw = pid_inte_yaw;

                // Throttle calculations (cross configuration)
                float motor1_throttle = throttle + pid_roll + pid_pitch + pid_yaw;
                float motor2_throttle = throttle - pid_roll + pid_pitch - pid_yaw;
                float motor3_throttle = throttle - pid_roll - pid_pitch + pid_yaw;
                float motor4_throttle = throttle + pid_roll - pid_pitch - pid_yaw;

                // Enforce throttle limits
                float motor1_ns = ((motor1_throttle > 0.0f) ? (motor1_throttle + 1.0f) : 1.0f);
                float motor2_ns = ((motor2_throttle > 0.0f) ? (motor2_throttle + 1.0f) : 1.0f);
                float motor3_ns = ((motor3_throttle > 0.0f) ? (motor3_throttle + 1.0f) : 1.0f);
                float motor4_ns = ((motor4_throttle > 0.0f) ? (motor4_throttle + 1.0f) : 1.0f);

                motor1_ns = (motor1_ns > 2.0f) ? 2.0f : motor1_ns;
                motor2_ns = (motor2_ns > 2.0f) ? 2.0f : motor2_ns;
                motor3_ns = (motor3_ns > 2.0f) ? 2.0f : motor3_ns;
                motor4_ns = (motor4_ns > 2.0f) ? 2.0f : motor4_ns;

                // Calculate PWM levels
                motor1_pwm_level = motor1_ns * 0.25f * wrap_num;
                motor2_pwm_level = motor2_ns * 0.25f * wrap_num;
                motor3_pwm_level = motor3_ns * 0.25f * wrap_num;
                motor4_pwm_level = motor4_ns * 0.25f * wrap_num;

                pwm_set_gpio_level(PIN_MOTOR1, motor1_pwm_level);
                pwm_set_gpio_level(PIN_MOTOR2, motor2_pwm_level);
                pwm_set_gpio_level(PIN_MOTOR3, motor3_pwm_level);
                pwm_set_gpio_level(PIN_MOTOR4, motor4_pwm_level);

                /* DEBUG PRINTS */
                // printf("Loop duration: %f seconds\n", ((float)time_us_64() - (float)loop_end_time) * 0.000001 + 0.004f);
                // printf("X: %f    Y: %f    Z: %f\n", normalised_gyro_values[0], normalised_gyro_values[1], normalised_gyro_values[2]);
                // printf("X: %f    Y: %f    Z: %f\n", normalised_accel_values[0], normalised_accel_values[1], normalised_accel_values[2]);
                // printf("%f    %f    %f    %f    %f    %f\n", normalised_rc_values[0], normalised_rc_values[1], normalised_rc_values[2], normalised_rc_values[3], normalised_rc_values[4], normalised_rc_values[5]);
                // printf("%d  %d  %d  %d\n", motor1_pwm_level, motor2_pwm_level, motor3_pwm_level, motor4_pwm_level);
            }
            else { // Motors not armed
                rc_read();

                // Arm motors at low throttle
                if (normalised_rc_values[RC_SWA] > 0.5f) {
                    if (normalised_rc_values[RC_THROTTLE] < 0.05f) {
                        motors_are_armed = true;
                        uint64_t spin_up_delay = time_us_64() + 400000;

                        while (time_us_64() < spin_up_delay) {
                            pwm_set_gpio_level(PIN_MOTOR1, esc_max);
                            pwm_set_gpio_level(PIN_MOTOR2, esc_max);
                            pwm_set_gpio_level(PIN_MOTOR3, esc_max);
                            pwm_set_gpio_level(PIN_MOTOR4, esc_max);
                        };

                        spin_up_delay = time_us_64() + 400000;

                        while (time_us_64() < spin_up_delay) {
                            pwm_set_gpio_level(PIN_MOTOR1, 0);
                            pwm_set_gpio_level(PIN_MOTOR2, 0);
                            pwm_set_gpio_level(PIN_MOTOR3, 0);
                            pwm_set_gpio_level(PIN_MOTOR4, 0);
                        };

                        prev_error_roll = 0.0f;
                        prev_error_pitch = 0.0f;
                        prev_error_yaw = 0.0f;
                        prev_integ_roll = 0.0f;
                        prev_integ_pitch = 0.0f;
                        prev_integ_yaw = 0.0f;
                        pid_error_roll = 0.0f;
                        pid_error_pitch = 0.0f;
                        pid_error_yaw = 0.0f;
                    }
                }
                else {
                    pwm_set_gpio_level(PIN_MOTOR1, 0);
                    pwm_set_gpio_level(PIN_MOTOR2, 0);
                    pwm_set_gpio_level(PIN_MOTOR3, 0);
                    pwm_set_gpio_level(PIN_MOTOR4, 0);
                }
            }

            while (time_us_64() < loop_end_time); // Do nothing until 4 ms has passed since loop start
        } // End of main loop
    }
    else {
        printf("ERROR >>>>   Setup failed in %f seconds, exiting.\n\n", ((time_us_64() - start_timestamp) * 0.000001f - 5.0f));

        /* Only uncomment this when storing in flash */
        // while (true) {
        //     cyw43_arch_gpio_put(PIN_LED, 1);
        //     sleep_ms(250);
        //     cyw43_arch_gpio_put(PIN_LED, 0);
        //     sleep_ms(250);
        // }
    }
}
