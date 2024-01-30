#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"


////////////////// Defines //////////////////

#define MIN_THROTTLE 0.07
#define MAX_THROTTLE 0.80

#define GYRO_RANGE 250
#define ACCEL_RANGE 4

#define IMU i2c0

#define RC_THROTTLE 2
#define RC_ROLL 0
#define RC_PITCH 1
#define RC_YAW 3
#define RC_SWA 4
#define RC_SWB 5

#define GYRO_ROLL 0
#define GYRO_PITCH 1
#define GYRO_YAW 2


////////////////// Settings //////////////////



////////////////// PID //////////////////


static const float MAX_ROLL_RATE = 30.0;
static const float MAX_PITCH_RATE = 30.0;
static const float 

static const float 
static const float 
static const float 

static const float 
static const float 
static const float 

static const float 
static const float 
static const float 


////////////////// Constants //////////////////

static const int MIN_THROTTLE_RATE = MIN_THROTTLE;
static const int MAX_THROTTLE_RATE = MAX_THROTTLE;

static const float GYRO_SCALE_MULTIPLIER = GYRO_RANGE/32767;
static const float ACCEL_SCALE_MULTIPLIER = ACCEL_RANGE/32767;

static const int IMU_I2C_ADDRESS = 104;
static const int IMU_REG_SMPLRT_DIV = 25;
static const int IMU_REG_PWR_MGMT1 = 107;
static const int IMU_REG_WHO_AM_I = 117;
static const int IMU_REG_CONFIG = 26; 
static const int IMU_REG_GYRO_CONFIG = 27;
static const int IMU_REG_ACCE_CONFIG = 28;
static const int IMU_REG_ACCE_X_HI = 59;
static const int IMU_REG_ACCE_X_LO = 60;
static const int IMU_REG_ACCE_Y_HI = 61; 
static const int IMU_REG_ACCE_Y_LO = 62;
static const int IMU_REG_ACCE_Z_HI = 63;
static const int IMU_REG_ACCE_Z_LO = 64;
static const int IMU_REG_TEMP_HI = 65;
static const int IMU_REG_TEMP_LO = 66;
static const int IMU_REG_GYRO_X_HI = 67;
static const int IMU_REG_GYRO_X_LO = 68;
static const int IMU_REG_GYRO_Y_HI = 69;
static const int IMU_REG_GYRO_Y_LO = 70;
static const int IMU_REG_GYRO_Z_HI = 71;
static const int IMU_REG_GYRO_Z_LO = 72;


////////////////// Global variables //////////////////

volatile float prev_error_roll = 0.0;
volatile float prev_error_pitch = 0.0;
volatile float prev_error_yaw = 0.0;
volatile float prev_integ_roll = 0.0;
volatile float prev_integ_pitch = 0.0;
volatile float prev_integ_yaw = 0.0;


int setup() {
    
    stdio_init_all();
}


void loop() {
    unsigned long loop_start_timestamp = micros();
    printf("Hello World!");
    while (micros() - loop_start_timestamp < 4000000); // Do nothing until 4 ms has passed since loop start
}