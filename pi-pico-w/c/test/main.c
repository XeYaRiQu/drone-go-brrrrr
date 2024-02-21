/*
 * Only change values in Settings.
*/

////////////////// Includes //////////////////

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/cyw43_arch.h" // WiFi chip on Pico W
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "hardware/uart.h"
#include "hardware/clocks.h"


////////////////// Settings //////////////////

#define MIN_THROTTLE 0.07
#define MAX_THROTTLE 0.80
#define THROTTLE_RANGE    (MAX_THROTTLE-MIN_THROTTLE)
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
#define PIN_LED     0  // The LED pin is tied to CYW43
#define PIN_RC_RX   5  // UART1
#define PIN_RC_TX   4  // UART1
#define PIN_IMU_SDA 12 // I2C0
#define PIN_IMU_SCL 13 // I2C0
#define PIN_MOTOR1  2  // PWM channel: 1A
#define PIN_MOTOR2  28 // PWM channel: 6A
#define PIN_MOTOR3  16 // PWM channel: 0A
#define PIN_MOTOR4  15 // PWM channel: 7B

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


////////////////// IMU //////////////////

#define IMU_I2C_ADDRESS  104
#define IMU_SMPLRT_DIV   25
#define IMU_PWR_MGMT1    107
#define IMU_WHO_AM_I     117
#define IMU_CONFIG       26
#define IMU_GYRO_CONFIG  27
#define IMU_ACCEL_CONFIG 28
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

#define RESET_ALL_BYTE         128
#define WAKE_TEMP_DISABLE_BYTE 9
#define WAKE_TEMP_ENABLE_BYTE  1


////////////////// Global variables //////////////////

volatile float prev_error_roll = 0.0;
volatile float prev_error_pitch = 0.0;
volatile float prev_error_yaw = 0.0;
volatile float prev_integ_roll = 0.0;
volatile float prev_integ_pitch = 0.0;
volatile float prev_integ_yaw = 0.0;

float gyro_multiplier, accel_multiplier;
int gyro_config_byte, accel_config_byte;

bool motors_are_armed = false;
float normalised_rc_values[6];
float normalised_gyro_values[3];
float gyro_x_bias, gyro_y_bias, gyro_z_bias;


////////////////// Functions //////////////////

void write_register(i2c_inst_t *i2c, int addr, int reg, int val) {
    uint8_t buffer[] = {(uint8_t) reg, (uint8_t) val};
    i2c_write_blocking(i2c, (uint8_t) addr, buffer, 2, false);
}


int read_register(i2c_inst_t *i2c, int addr, int reg) {
    uint8_t *val = reg;
    int *readout;
    i2c_write_blocking(i2c, addr, val, 1, true);
    i2c_read_blocking(i2c, addr, readout, 1, false);
    return *readout;
}


int mpu6050_init() {
    int fail_flag = 0;

    switch (GYRO_RANGE) {
        case RANGE_250DPS:
            gyro_multiplier = 250.0/32768.0;
            gyro_config_byte = 0x00;
            break;
        case RANGE_500DPS:
            gyro_multiplier = 500.0/32768.0;
            gyro_config_byte = 0x08;
            break;
        case RANGE_1000DPS:
            gyro_multiplier = 1000.0/32768.0;
            gyro_config_byte = 0x10;
            break;
        case RANGE_2000DPS:
            gyro_multiplier = 2000.0/32768.0;
            gyro_config_byte = 0x18;
            break;
    }

    switch (ACCEL_RANGE) {
        case RANGE_2G:
            accel_multiplier = 2.0/32768.0;
            accel_config_byte = 0x00;
            break;
        case RANGE_4G:
            accel_multiplier = 4.0/32768.0;
            accel_config_byte = 0x08;
            break;
        case RANGE_8G:
            accel_multiplier = 8.0/32768.0;
            accel_config_byte = 0x10;
            break;
        case RANGE_16G:
            accel_multiplier = 16.0/32768.0;
            accel_config_byte = 0x18;
            break;
    }

    // Reset all registers
    write_register(i2c0, IMU_I2C_ADDRESS, IMU_PWR_MGMT1, RESET_ALL_BYTE);
    sleep_ms(10);

    // Wake, disable temperature sensor
    write_register(i2c0, IMU_I2C_ADDRESS, IMU_PWR_MGMT1, WAKE_TEMP_DISABLE_BYTE);
    sleep_ms(10);

    // Set sensor sample rate to 250 Hz (Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV))
    write_register(i2c0, IMU_I2C_ADDRESS, IMU_SMPLRT_DIV, 0x03);
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
    if (read_register(i2c0, IMU_I2C_ADDRESS, IMU_PWR_MGMT1) != WAKE_TEMP_DISABLE_BYTE) {
        fail_flag = 1;
        printf("ERROR >>>>   Verify MPU-6050 register: PWR_MGMT_1 --> FAIL\n\n");
    }

    if (read_register(i2c0, IMU_I2C_ADDRESS, IMU_SMPLRT_DIV) != 0x03) {
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
    uint8_t *accel_xout_h = IMU_ACCE_X_H;
    uint8_t *gyro_xout_h = IMU_GYRO_X_H;

    // Get raw accelerometer data
    i2c_write_blocking(i2c_default, IMU_I2C_ADDRESS, accel_xout_h, 1, true);
    i2c_read_blocking(i2c_default, IMU_I2C_ADDRESS, accel_buffer, 6, false);

    // Get raw gyro data
    i2c_write_blocking(i2c_default, IMU_I2C_ADDRESS, gyro_xout_h, 1, true);
    i2c_read_blocking(i2c_default, IMU_I2C_ADDRESS, gyro_buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        raw_accel_data[i] = (accel_buffer[i * 2] << 8 | accel_buffer[(i * 2) + 1]);
        raw_gyro_data[i] = (gyro_buffer[i * 2] << 8 | gyro_buffer[(i * 2) + 1]);
    }

    uint16_t accel_x = raw_accel_data[0];
    uint16_t accel_y = raw_accel_data[1];
    uint16_t accel_z = raw_accel_data[2];
    uint16_t gyro_x = raw_gyro_data[0];
    uint16_t gyro_y = raw_gyro_data[1];
    uint16_t gyro_z = raw_gyro_data[2];

    /* DOUBLE CHECK THE SENSOR ORIENTATION TO COMPLY WITH NORTH-EAST-DOWN (NED) */
    // Normalise roll
    if (gyro_x > 32767) {
        normalised_gyro_values[GYRO_ROLL] = (gyro_x - 65536) * gyro_multiplier - gyro_x_bias;
    } else {
        normalised_gyro_values[GYRO_ROLL] = gyro_x * gyro_multiplier - gyro_x_bias;
    }

    // Normalise pitch
    if (gyro_y > 32767) {
        normalised_gyro_values[GYRO_PITCH] = (gyro_y - 65536) * gyro_multiplier - gyro_y_bias;
    } else {
        normalised_gyro_values[GYRO_PITCH] = gyro_y * gyro_multiplier - gyro_y_bias;
    }

    // Normalise yaw
    if (gyro_z > 32767) {
        normalised_gyro_values[GYRO_YAW] = (gyro_z - 65536) * gyro_multiplier - gyro_z_bias;
    } else {
        normalised_gyro_values[GYRO_YAW] = gyro_z * gyro_multiplier - gyro_z_bias;
    }

    // Add normalisation for accelerometer values

}


void mpu_6050_cali() {
    float gyro_bias_x_data[GYRO_ARRAY_SIZE] = {0.0f};
    float gyro_bias_y_data[GYRO_ARRAY_SIZE] = {0.0f};
    float gyro_bias_z_data[GYRO_ARRAY_SIZE] = {0.0f};
    int gyro_bias_data_points = 0;
    uint32_t gyro_bias_duration = time_us_64() + 5000000;
    
    printf("INFO  >>>>   Calculating gyroscope bias. Keep vehicle still.\n");

    while (time_us_64() < gyro_bias_duration) {
        imu_read();
        gyro_bias_x_data[gyro_bias_data_points] = normalised_gyro_values[0];
        gyro_bias_y_data[gyro_bias_data_points] = normalised_gyro_values[1];
        gyro_bias_z_data[gyro_bias_data_points] = normalised_gyro_values[2];
        gyro_bias_data_points++;
        sleep_ms(50);
    }

    gyro_offset_bias[GYRO_X_OFFSET] = 0.0f;
    gyro_offset_bias[GYRO_Y_OFFSET] = 0.0f;
    gyro_offset_bias[GYRO_Z_OFFSET] = 0.0f;

    //Summing data
    for (int i = 0; i < gyro_bias_data_points; i++) {
        gyro_offset_bias[GYRO_X_OFFSET] += gyro_bias_x_data[i];
        gyro_offset_bias[GYRO_Y_OFFSET] += gyro_bias_y_data[i];
        gyro_offset_bias[GYRO_Z_OFFSET] += gyro_bias_z_data[i];
    }

    //Dividing by number of data points to get average
    gyro_offset_bias[GYRO_X_OFFSET] /= gyro_bias_data_points;
    gyro_offset_bias[GYRO_Y_OFFSET] /= gyro_bias_data_points;
    gyro_offset_bias[GYRO_Z_OFFSET] /= gyro_bias_data_points;

    printf("INFO  >>>>   Gyroscope offsets saved. Offsets:\n");
    printf("INFO  >>>>   X: %f\n", gyro_offset_bias[GYRO_X_OFFSET]);
    printf("INFO  >>>>   Y: %f\n", gyro_offset_bias[GYRO_Y_OFFSET]);
    printf("INFO  >>>>   Z: %f\n", gyro_offset_bias[GYRO_Z_OFFSET]);
}


////////////////// Setup //////////////////

int setup() {
    int fail_flag = 0;
    uint64_t setup_start = time_us_64();
    uint64_t setup_delay = setup_start + 5000000;
    stdio_init_all();

    while (time_us_64()  < setup_delay); // Delay for serial monitor to establish connection

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

    // Perform IMU self-test and calibration
    mpu_6050_cali();

    return fail_flag;

    //Motors Setup

    printf("INFO  >>>> Setting up motors \n");

    gpio_set_function(PIN_MOTOR1, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR2, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR3, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR3, GPIO_FUNC_PWM);

    int slice_num1 =  pwm_gpio_to_slice_num(PIN_MOTOR1);
    int slice_num2 =  pwm_gpio_to_slice_num(PIN_MOTOR2);
    int slice_num3 =  pwm_gpio_to_slice_num(PIN_MOTOR3);
    int slice_num4 =  pwm_gpio_to_slice_num(PIN_MOTOR4);

    pwm_set_enabled(slice_num1, true);
    pwm_set_enabled(slice_num2, true);
    pwm_set_enabled(slice_num3, true);
    pwm_set_enabled(slice_num4, true);

    pwm_set_clkdiv(slice_num1, 100.0 );
    pwm_set_clkdiv(slice_num2, 100.0 );
    pwm_set_clkdiv(slice_num3, 100.0 );
    pwm_set_clkdiv(slice_num4, 100.0 ); //so now the clock runs at 125kHz instead of 125MHz

    pwm_set_wrap(slice_num1, 4999); //For 250Hz freq, wrap num = 125000/250  - 1 = 4999
    pwm_set_wrap(slice_num2, 4999); 
    pwm_set_wrap(slice_num3, 4999); 
    pwm_set_wrap(slice_num4, 4999); 
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
            uint8_t buffer[30];

            // Read start bytes
            uint8_t char1 = uart_getc(uart1);
            uint8_t char2 = uart_getc(uart1);

            // Validate start bytes
            if (char1 == 0x20 && char2 == 0x40) {
                // Read the rest of the data into the buffer
                uart_read_blocking(uart1, buffer, 30);

                // Calculate and set the checksum
                uint16_t checksum = 0xFF9F; // 0xFFFF - 0x20 - 0x40
                
                //Validating checksum
                for (int byte_index = 0; byte_index < 28; ++byte_index) {
                checksum -= buffer[byte_index];
                }

                if (checksum == (buffer[29] << 8) | buffer[28]) { // Convert to big endian
                // Process raw values
                    int raw_rc_values[6];
                    for (int channel = 0; channel < 6; ++channel) {
                    raw_rc_values[channel] = (buffer[channel * 2 + 1] << 8) + buffer[channel * 2];
                    }
                   
                    // Normalize data
                    normalised_rc_values[RC_THROTTLE] = (float)(raw_rc_values[RC_THROTTLE] * 0.001 - 1); // Normalize from 1000-2000 to 0.0-1.0
                    normalised_rc_values[RC_ROLL] = (float)((raw_rc_values[RC_ROLL] - 1500) * 0.002);    // Normalize from 1000-2000 to -1-1
                    normalised_rc_values[RC_PITCH] = (float)((raw_rc_values[RC_PITCH] - 1500) * 0.002);  // Normalize from 1000-2000 to -1-1
                    normalised_rc_values[RC_YAW] = (float)-((raw_rc_values[RC_YAW] - 1500) * 0.002);      // Normalize from 1000-2000 to -1-1
                    normalised_rc_values[RC_SWA] = (float)(raw_rc_values[RC_SWA] * 0.001 - 1);    // Normalize from 1000-2000 to 0-1
                    normalised_rc_values[RC_SWB] = (float)(raw_rc_values[RC_SWB] * 0.001 - 1);    // Normalize from 1000-2000 to 0-1
                    
                    break;
                }
            }
        }
    }
}


////////////////// Main //////////////////

void main() {
    uint64_t start_timestamp = time_us_64();

    printf("INFO  >>>>   Executing setup sequence.\n\n");
    if (setup() == 0) {

        printf("INFO  >>>>   Setup completed in %f seconds, looping.\n\n", ((double)(time_us_64() - start_timestamp)/1000000 - 5));
        uint64_t prev_pid_timestamp = time_us_64();
        ////////////////// Loop //////////////////
        while (true) {
            start_timestamp = time_us_64();
            if (motors_are_armed == true) {
                rc_read();

                if (normalised_rc_values[RC_SWA] == 0) {  // does this need to be SWA or SWB? need to confirm
                    if (normalised_rc_values[RC_THROTTLE] < 0.05){   //so that motors stop even if throttle control is not exactly = 0
                        pwm_set_gpio_level(PIN_MOTOR1, 0); //motors running at 0% duty cycle. motors not rotating
                        pwm_set_gpio_level(PIN_MOTOR2, 0);
                        pwm_set_gpio_level(PIN_MOTOR3, 0);
                        pwm_set_gpio_level(PIN_MOTOR4, 0);
                        motors_are_armed = false;
                    }
                }

                imu_read();
                float desired_throttle_rate = normalised_rc_values[RC_THROTTLE];
                float desired_pitch_rate = normalised_rc_values[RC_PITCH];
                float desired_roll_rate = normalised_rc_values[RC_ROLL];
                float desired_yaw_rate = normalised_rc_values[RC_YAW];

                // Error calculations (desired - actual)
                float pid_error_roll = desired_roll_rate * MAX_ROLL_RATE - normalised_gyro_values[GYRO_ROLL];
                float pid_error_pitch = desired_pitch_rate * MAX_PITCH_RATE - normalised_gyro_values[GYRO_PITCH];
                float pid_error_yaw = desired_yaw_rate * MAX_YAW_RATE - normalised_gyro_values[GYRO_YAW];

                // Proportion calculations
                float pid_prop_roll = pid_error_roll * KP_ROLL;
                float pid_prop_pitch = pid_error_pitch * KP_PITCH;
                float pid_prop_yaw = pid_error_yaw * KP_YAW;

                // Calculate time elapsed since previous PID calculations
                float pid_cycle_time = 1.0 / (((float)(time_us_64() - prev_pid_timestamp)) * 0.000001);

                // Integral calculations
                float pid_inte_roll = pid_error_roll * KI_ROLL * pid_cycle_time + prev_integ_roll;
                float pid_inte_pitch = pid_error_pitch * KI_PITCH * pid_cycle_time + prev_integ_pitch;
                float pid_inte_yaw = pid_error_yaw * KI_YAW * pid_cycle_time + prev_integ_yaw;

                // Derivative calculations
                float pid_deri_roll = (pid_error_roll - prev_error_roll) * KD_ROLL * pid_cycle_time;
                float pid_deri_pitch = (pid_error_pitch - prev_error_pitch) * KD_PITCH * pid_cycle_time;
                float pid_deri_yaw = (pid_error_yaw - prev_error_yaw) * KD_YAW * pid_cycle_time;

                // Capture end timestamp for current PID loop
                prev_pid_timestamp = time_us_64();

                float throttle_rate = desired_throttle_rate * THROTTLE_RANGE + MIN_THROTTLE;
                float pid_roll = pid_prop_roll + pid_inte_roll + pid_deri_roll;
                float pid_pitch = pid_prop_pitch + pid_inte_pitch + pid_deri_pitch;
                float pid_yaw = pid_prop_yaw + pid_inte_yaw + pid_deri_yaw;

                // Throttle calculations (cross configuration)
                float motor1_throttle = throttle_rate - pid_roll + pid_pitch + pid_yaw;
                float motor2_throttle = throttle_rate + pid_roll + pid_pitch - pid_yaw;
                float motor3_throttle = throttle_rate + pid_roll - pid_pitch + pid_yaw;
                float motor4_throttle = throttle_rate - pid_roll - pid_pitch - pid_yaw;

                // Save PID values for subsequent calculations
                prev_error_roll = pid_error_roll;
                prev_error_pitch = pid_error_pitch;
                prev_error_yaw = pid_error_yaw;
                prev_integ_roll = pid_inte_roll;
                prev_integ_pitch = pid_inte_pitch;
                prev_integ_yaw = pid_inte_yaw;

                //Calculating duty cycle
                float motor1_temp = ((motor1_throttle * 1000000 > 0) ? motor1_throttle * 1000000 : 0) + 1000000;
                float motor2_temp = ((motor2_throttle * 1000000 > 0) ? motor2_throttle * 1000000 : 0) + 1000000;
                float motor3_temp = ((motor3_throttle * 1000000 > 0) ? motor3_throttle * 1000000 : 0) + 1000000;
                float motor4_temp = ((motor4_throttle * 1000000 > 0) ? motor4_throttle * 1000000 : 0) + 1000000;

                int motor1_ns = (motor1_temp > 2000000) ? 2000000 : (int)motor1_temp;
                int motor2_ns = (motor2_temp > 2000000) ? 2000000 : (int)motor2_temp;
                int motor3_ns = (motor3_temp > 2000000) ? 2000000 : (int)motor3_temp;
                int motor4_ns = (motor4_temp > 2000000) ? 2000000 : (int)motor4_temp;

                float motor1_dutycycle = motor1_ns/4000000;  //motor1_ns is between 1000000 and 2000000. 25%-50% duty cycle
                float motor2_dutycycle = motor2_ns/4000000;
                float motor3_dutycycle = motor3_ns/4000000;
                float motor4_dutycycle = motor4_ns/4000000;

                //Convert duty cycle to setpoint for gpio_set_level. setpoint = wrap number * duty cycle
                u_int16_t setpoint_motor1 = 4999 * motor1_dutycycle;
                u_int16_t setpoint_motor2 = 4999 * motor2_dutycycle;
                u_int16_t setpoint_motor3 = 4999 * motor3_dutycycle;
                u_int16_t setpoint_motor4 = 4999 * motor4_dutycycle;

                pwm_set_gpio_level(PIN_MOTOR1, setpoint_motor1);
                pwm_set_gpio_level(PIN_MOTOR2, setpoint_motor2);
                pwm_set_gpio_level(PIN_MOTOR3, setpoint_motor3);
                pwm_set_gpio_level(PIN_MOTOR4, setpoint_motor4);
            }
            else {
                rc_read();
                if (normalised_rc_values[RC_SWA] == 1) { //SWA or SWB
                    if (normalised_rc_values[RC_THROTTLE] == 0.0) {  // again, is this condition correct?
                        motors_are_armed = true;
                        uint16_t startup_pwm = (uint16_t)(4999 * 0.25f);//setting 25% dutycycle
                        uint64_t spin_up_delay= time_us_64() + 1000;

                        while (time_us_64() < spin_up_delay) {
                            pwm_set_gpio_level(PIN_MOTOR1, startup_pwm); //motors running at 25% duty cycle
                            pwm_set_gpio_level(PIN_MOTOR2, startup_pwm);
                            pwm_set_gpio_level(PIN_MOTOR3, startup_pwm);
                            pwm_set_gpio_level(PIN_MOTOR4, startup_pwm);
                        }
                        float prev_pid_error_roll = 0.0;
                        float prev_pid_error_pitch = 0.0;
                        float prev_pid_error_yaw = 0.0;
                        float prev_pid_inte_roll = 0.0;
                        float prev_pid_inte_pitch = 0.0;
                        float prev_pid_inte_yaw = 0.0;

                        prev_pid_timestamp = time_us_64();
                    }
                }
                else {
                    pwm_set_gpio_level(PIN_MOTOR1, 0); //motors running at 0% duty cycle. motors not rotating
                    pwm_set_gpio_level(PIN_MOTOR2, 0);
                    pwm_set_gpio_level(PIN_MOTOR3, 0);
                    pwm_set_gpio_level(PIN_MOTOR4, 0);
                }
            }
            while (time_us_64() - start_timestamp < 4000); // Do nothing until 4 ms has passed since loop start
        }
    }
    else {
        printf("ERROR >>>> -> SETUP FAIL\n");
        printf("ERROR >>>>   Setup failed in %f seconds, exiting.\n\n", ((double)(time_us_64() - start_timestamp)/1000000 - 5));
    }
}
