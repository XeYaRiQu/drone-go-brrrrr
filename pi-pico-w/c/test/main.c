/**
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

int GYRO_RANGE = RANGE_1000DPS;
int ACCEL_RANGE = RANGE_4G;
int lpf_config_byte = 2;
//lpf config byte, acc lpf, gyro lpf --- 0,260,256 :: 1,184,188 :: 2,94,98 :: 3,44,42 :: 4,21,20 ::5,10,10 :: 6,5,5




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
#define IMU_REG_GYRO_X_HI    67
#define IMU_REG_GYRO_X_LO    68
#define IMU_REG_GYRO_Y_HI    69
#define IMU_REG_GYRO_Y_LO    70
#define IMU_REG_GYRO_Z_HI    71
#define IMU_REG_GYRO_Z_LO    72


////////////////// Constants //////////////////

static const int MIN_THROTTLE = MIN_THROTTLE_RATE;
static const int MAX_THROTTLE = MAX_THROTTLE_RATE;
int THROTTLE_RANGE = MAX_THROTTLE_RATE - MIN_THROTTLE_RATE;

////////////////// Global variables //////////////////

volatile float prev_error_roll = 0.0;
volatile float prev_error_pitch = 0.0;
volatile float prev_error_yaw = 0.0;
volatile float prev_integ_roll = 0.0;
volatile float prev_integ_pitch = 0.0;
volatile float prev_integ_yaw = 0.0;

float gyro_multiplier, accel_multiplier;
int gyro_config_byte, accel_config_byte;

#define GYRO_X_OFFSET 0
#define GYRO_Y_OFFSET 1
#define GYRO_Z_OFFSET 2
#define GYRO_ARRAY_SIZE 101

bool motors_are_armed = false;
float normalised_rc_values[6] = {0.0f};
float normalised_gyro_values[3] = {0.0f};
float gyro_offset_bias[3] = {0.0f};

#define RC_BUFFER 30
#define UART_ID uart1

////////////////// Functions //////////////////

int mpu6050_init() {
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
#ifdef i2c_default
// Function to write a byte to IMU registers using I2C

void writeRegister(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    i2c_write_blocking(i2c_default, IMU_I2C_ADDRESS, buffer, 2, false);
};

//Function to verify if gyro registers have been written correctly

bool verifySetting(uint8_t address, uint8_t reg, uint8_t expected_value, const char *success_msg, const char *fail_msg) {
    uint8_t data;
    i2c_read_blocking(i2c_default, address, reg, 1, false);
    if (data == expected_value) {
        printf("INFO  >>>>   %s -> SUCCESS\n", success_msg);
        return true;
    } else {
        printf("ERROR >>>>   %s -> FAIL\n", fail_msg);
        return false;
    }
}


////////////////// Setup //////////////////

int setup() {
    int fail_flag = 0;
    uint64_t setup_start = time_us_64();
    uint64_t setup_delay = setup_start + 5000000;
    stdio_init_all();

    while (time_us_64()  < setup_delay);

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
    //i2c_inst_t *i2c = i2c0; using i2c_default instead

    if (i2c_init(i2c_default, 400000) == 400000) {
        
        gpio_set_function(PIN_IMU_SCL, GPIO_FUNC_I2C);
        gpio_set_function(PIN_IMU_SDA, GPIO_FUNC_I2C);
        gpio_pull_up(PIN_IMU_SCL);
        gpio_pull_up(PIN_IMU_SDA);
        // Make the I2C pins available to picotool (copied from mpu6050_i2c.c in pico-examples)
        bi_decl(bi_2pins_with_func(PIN_IMU_SDA, PIN_IMU_SCL, GPIO_FUNC_I2C));
        printf("INFO  >>>>   Initialise I2C --> SUCCESS\n\n");
        i2c_set_slave_mode(i2c_default, false, IMU_I2C_ADDRESS); //master mode set as mpu6050 (slave) needs to be configured by microcontroller (master)
    }
    else {
        fail_flag = 1;
        printf("INFO  >>>>   Initialise I2C --> FAIL\n\n");
    }

    return fail_flag;
    //Writing into registers
    // Reset all registers
    writeRegister(IMU_PWR_MGMT1, 0x80);
    sleep_ms(100);

    // Wake, disable temperature sensor
    writeRegister(IMU_PWR_MGMT1, 0x09);
    sleep_ms(100);

    // Set sensor sample rate to 250 Hz
    writeRegister(IMU_SMPLRT_DIV, 0x03);
    sleep_ms(100);

    // Set accelerometer LPF and gyroscope LPF 
    writeRegister(IMU_CONFIG, lpf_config_byte);
    sleep_ms(100);

    // Set gyroscope scale (in dps)
    writeRegister(IMU_GYRO_CONFIG, gyro_config_byte);

    printf("INFO  >>>>   MPU-6050 setup --> SUCCESS\n");

    //Reading from registers to check if values are updated correctly

    bool error_raised_flag = false;
    

    // Who am I check
    if (!verifySetting(IMU_I2C_ADDRESS, IMU_WHO_AM_I, IMU_I2C_ADDRESS, "MPU-6050 verify WHO_AM_I", "MPU-6050 WHO_AM_I read error.")) {
        error_raised_flag = true;
    }

    // Sleep and temperature sensor disabled check
    if (!verifySetting(IMU_I2C_ADDRESS, IMU_PWR_MGMT1, 9, "MPU-6050 verify IMU_REG_PWR_MGMT1", "MPU-6050 IMU_REG_PWR_MGMT1 not set.")) {
        error_raised_flag = true;
    }

    // Low pass filter check
    if (!verifySetting(IMU_I2C_ADDRESS, IMU_CONFIG, lpf_config_byte, "MPU-6050 verify IMU_REG_CONFIG", "MPU-6050 IMU_REG_CONFIG not set.")) {
        error_raised_flag = true;
    }

    // Sample rate check
    if (!verifySetting(IMU_I2C_ADDRESS, IMU_SMPLRT_DIV, 3, "MPU-6050 verify IMU_REG_SMPLRT_DIV", "MPU-6050 IMU_REG_SMPLRT_DIV not set.")) {
        error_raised_flag = true;
    }

    // Gyroscope scale check
    if (!verifySetting(IMU_I2C_ADDRESS, IMU_GYRO_CONFIG, gyro_config_byte, "MPU-6050 verify IMU_REG_GYRO_CONFIG", "MPU-6050 IMU_REG_GYRO_CONFIG not set.")) {
        error_raised_flag = true;
    }

    if (error_raised_flag) {
        printf("ERROR >>>>   MPU-6050 verify settings -> FAIL\n");
    } else {
        printf("INFO  >>>>   MPU-6050 verify settings -> SUCCESS\n");
    }


    // Cleanup
    //i2c_deinit(i2c_default); ---- not deinitialising because i2c will be used in imu_read function as well.

    calc_gyro_bias();

    if  (!error_raised_flag){
        return 0;
    } else {
        return 1;
    }

}

void calc_gyro_bias() {
    float gyro_bias_x_data[GYRO_ARRAY_SIZE] = {0.0f};
    float gyro_bias_y_data[GYRO_ARRAY_SIZE] = {0.0f};
    float gyro_bias_z_data[GYRO_ARRAY_SIZE] = {0.0f};
    int gyro_bias_data_points = 0;
    uint32_t gyro_bias_duration = time_ms() + 5000;
    
    printf("INFO  >>>>   Calculating gyroscope bias. Keep vehicle still.\n");

    while (time_ms() < gyro_bias_duration) {
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
        if (uart_is_readable(UART_ID)) {
            uint8_t buffer[RC_BUFFER];

            // Read start bytes
            uint8_t char1 = uart_getc(UART_ID);
            uint8_t char2 = uart_getc(UART_ID);

            // Validate start bytes
            if (char1 == 0x20 && char2 == 0x40) {
                // Read the rest of the data into the buffer
                uart_read_blocking(UART_ID, buffer, RC_BUFFER);

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


void imu_read() {
    /*
    The IMU measurements are 16-bit 2's complement values ranging from -32768
    to 32767 which needs to be multiplied by the scale multipliers to obtain the
    physical values. These scale multipliers change depending on the range set
    in their respective register configs. The high byte is read before the low
    byte for each measurement.
    */
 

    //The code reads the high and low bytes separately and combines them to form the 16-bit values 
    //for X, Y, and Z axes.
    uint8_t x_hi = i2c_read_blocking(i2c_default, IMU_I2C_ADDRESS, IMU_REG_GYRO_X_HI, 1, false);
    uint8_t x_lo = i2c_read_blocking(i2c_default, IMU_I2C_ADDRESS, IMU_REG_GYRO_X_LO, 1, false);
    uint8_t y_hi = i2c_read_blocking(i2c_default, IMU_I2C_ADDRESS, IMU_REG_GYRO_Y_HI, 1, false);
    uint8_t y_lo = i2c_read_blocking(i2c_default, IMU_I2C_ADDRESS, IMU_REG_GYRO_Y_LO, 1, false);
    uint8_t z_hi = i2c_read_blocking(i2c_default, IMU_I2C_ADDRESS, IMU_REG_GYRO_Z_HI, 1, false);
    uint8_t z_lo = i2c_read_blocking(i2c_default, IMU_I2C_ADDRESS, IMU_REG_GYRO_Z_LO, 1, false);

    int x_value = (x_hi << 8) | x_lo; //x_value is 16 bit
    int y_value = (y_hi << 8) | y_lo;
    int z_value = (z_hi << 8) | z_lo;
    
    // Normalize X axis
    if (x_value > 32767) {
        normalised_gyro_values[GYRO_ROLL] = (x_value - 65536) * gyro_multiplier - gyro_offset_bias[GYRO_ROLL];
    } else {
        normalised_gyro_values[GYRO_ROLL] = x_value * gyro_multiplier - gyro_offset_bias[GYRO_ROLL];
    }

    // Normalize Y axis
    if (y_value > 32767) {
        normalised_gyro_values[GYRO_PITCH] = (y_value - 65536) * gyro_multiplier - gyro_offset_bias[GYRO_PITCH];
    } else {
        normalised_gyro_values[GYRO_PITCH] = y_value * gyro_multiplier - gyro_offset_bias[GYRO_PITCH];
    }

    // Normalize Z axis
    if (z_value > 32767) {
        normalised_gyro_values[GYRO_YAW] = (z_value - 65536) * gyro_multiplier - gyro_offset_bias[GYRO_YAW];
    } else {
        normalised_gyro_values[GYRO_YAW] = z_value * gyro_multiplier - gyro_offset_bias[GYRO_YAW];
    }


}
#endif

////////////////// Main //////////////////

void main() {
    uint64_t start_timestamp = time_us_64();

    printf("INFO  >>>>   Executing setup sequence.\n\n");
    if (setup()) {
        printf("ERROR >>>>   Setup failed in %f seconds, exiting.\n\n", ((double)(time_us_64() - start_timestamp)/1000000 - 5));
    }
    else {
        printf("INFO  >>>>   Setup completed in %f seconds, looping.\n\n", ((double)(time_us_64() - start_timestamp)/1000000 - 5));
        uint64_t prev_pid_timestamp = time_us_64();
        ////////////////// Loop //////////////////
        while (true) {
            start_timestamp = time_us_64();
            if (motors_are_armed == true) {
                rc_read();

                if (normalised_rc_values[RC_SWA] == 0) {
                    if (normalised_rc_values[RC_THROTTLE] == 0.0){
                        //deinit 4 motors
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
                prev_pid_timestamp = time_us_64();

            }
            
            else {

            }

            while (time_us_64() - start_timestamp < 4000); // Do nothing until 4 ms has passed since loop start
        }
    }
}