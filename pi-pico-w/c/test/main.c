/**
 * Only change values in Settings.
*/

////////////////// Includes //////////////////

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/cyw43_arch.h" // WiFi chip on Pico W
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "hardware/uart.h"
#include "hardware/clocks.h"


////////////////// Settings //////////////////

#define MIN_THROTTLE_RATE 0.07
#define MAX_THROTTLE_RATE 0.80
#define MAX_YAW_RATE      30.0
#define MAX_ROLL_RATE     30.0
#define MAX_PITCH_RATE    30.0

#define RC_THROTTLE 2
#define RC_ROLL     0
#define RC_PITCH    1
#define RC_YAW      3
#define RC_SWA      4
#define RC_SWB      5

#define GYRO_ROLL  0
#define GYRO_PITCH 1
#define GYRO_YAW   2

/* Pin placement */
#define PIN_LED     0 // The LED pin is tied to CYW43
#define PIN_RC_RX   5
#define PIN_RC_TX   4
#define PIN_IMU_SDA 12
#define PIN_IMU_SCL 13
#define PIN_MOTOR1  2
#define PIN_MOTOR2  28
#define PIN_MOTOR3  16
#define PIN_MOTOR4  15

enum GYRO_RANGE {
    RANGE_250DPS = 0,
    RANGE_500DPS,
    RANGE_1000DPS,
    RANGE_2000DPS
};

enum ACCEL_RANGE {
    RANGE_2G = 0,
    RANGE_4G,
    RANGE_8G,
    RANGE_16G
};

int GYRO_RANGE = RANGE_500DPS;
int ACCEL_RANGE = RANGE_4G;


////////////////// PID //////////////////

static const float KP_ROLL = 0.00043714285;
static const float KP_PITCH = 0.00043714285;
static const float KP_YAW = 0.001714287;

static const float KI_ROLL = 0.00255;
static const float KI_PITCH = 0.00255;
static const float KI_YAW = 0.003428571;

static const float KD_ROLL = 0.00002571429;
static const float KD_PITCH = 0.00002571429;
static const float KD_YAW = 0.0;

static const float I_LIMIT_POS = 100.0;
static const float I_LIMIT_NEG = -100.0;


////////////////// Defines //////////////////

#define IMU_I2C_ADDRESS 104
#define IMU_SMPLRT_DIV  25
#define IMU_PWR_MGMT1   107
#define IMU_WHO_AM_I    117
#define IMU_CONFIG      26
#define IMU_GYRO_CONFIG 27
#define IMU_ACCE_CONFIG 28
#define IMU_ACCE_X_H    59
#define IMU_ACCE_X_L    60
#define IMU_ACCE_Y_H    61
#define IMU_ACCE_Y_L    62
#define IMU_ACCE_Z_H    3
#define IMU_ACCE_Z_L    64
#define IMU_TEMP_HI     65
#define IMU_TEMP_LO     66
#define IMU_GYRO_X_H    67
#define IMU_GYRO_X_L    68
#define IMU_GYRO_Y_H    69
#define IMU_GYRO_Y_L    70
#define IMU_GYRO_Z_H    71
#define IMU_GYRO_Z_L    72


////////////////// Constants //////////////////

static const int MIN_THROTTLE = MIN_THROTTLE_RATE;
static const int MAX_THROTTLE = MAX_THROTTLE_RATE;


////////////////// Global variables //////////////////

volatile float prev_error_roll = 0.0;
volatile float prev_error_pitch = 0.0;
volatile float prev_error_yaw = 0.0;
volatile float prev_integ_roll = 0.0;
volatile float prev_integ_pitch = 0.0;
volatile float prev_integ_yaw = 0.0;

float gyro_multiplier, accel_multiplier;
int gyro_config_byte, accel_config_byte;


////////////////// Functions //////////////////

int mpu6050_init() {
    switch (GYRO_RANGE) {
        case RANGE_250DPS:
            gyro_multiplier = 250.0/32768.0;
            break;
        case RANGE_500DPS:
            gyro_multiplier = 500.0/32768.0;
            break;
        case RANGE_1000DPS:
            gyro_multiplier = 1000.0/32768.0;
            break;
        case RANGE_2000DPS:
            gyro_multiplier = 2000.0/32768.0;
            break;
    }

    switch (ACCEL_RANGE) {
        case RANGE_2G:
            accel_multiplier = 2.0/32768.0;
            break;
        case RANGE_4G:
            accel_multiplier = 4.0/32768.0;
            break;
        case RANGE_8G:
            accel_multiplier = 8.0/32768.0;
            break;
        case RANGE_16G:
            accel_multiplier = 16.0/32768.0;
            break;
    }
    return 0;
};

void read_gyro() {
    
};

////////////////// Setup //////////////////

int setup() {
    int fail_flag = 0;
    uint64_t setup_start = time_us_64();

    stdio_init_all();

    while (time_us_64() - setup_start < 5000000);

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

    return fail_flag;
}


////////////////// Main //////////////////

void main() {
    uint64_t start_timestamp = time_us_64();

    printf("INFO  >>>>   Executing setup sequence.\n\n");
    if (setup()) {
        printf("ERROR >>>>   Setup failed in %f seconds, exiting.\n\n", ((double)(time_us_64() - start_timestamp)/1000000 - 5));
    }
    else {
        printf("INFO  >>>>   Setup completed in %f seconds, looping.\n\n", ((double)(time_us_64() - start_timestamp)/1000000 - 5));

        ////////////////// Loop //////////////////
        while (true) {
            start_timestamp = time_us_64();

            

            while (time_us_64() - start_timestamp < 4000); // Do nothing until 4 ms has passed since loop start
        }
    }
}