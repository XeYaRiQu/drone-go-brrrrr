//from machine import Pin, UART, I2C, PWM, freq
//rc:UART = UART(1)

#include <stdio.h>
#include <math.h>
#include <time.h>

// General Settings
int gyro_range = 500; //250, 500, 1000, 2000 dps (IMU_REG_GYRO_CONFIG[3:4])
int acce_range = 4; // 2, 4, 8, 16 g (IMU_REG_ACCE_CONFIG[3:4])

float min_throttle_rate = 0.07f;
float max_throttle_rate = 0.80f;

//PID Settings
float max_roll_rate = 30.0f; //degrees per second
float max_pitch_rate = 30.0f; //degrees per second
float max_yaw_rate = 60.0f; //degrees per second

float pid_kp_roll = 0.00043714285f;
float pid_kp_pitch = 0.00043714285f;
float pid_kp_yaw = 0.001714287f;

float pid_ki_roll = 0.00255f;
float pid_ki_pitch = 0.00255f;
float pid_ki_yaw = 0.003428571f;

float pid_kd_roll = 0.00002571429f;
float pid_kd_pitch = 0.00002571429f;
float pid_kd_yaw = 0.0f;

float pid_integral_limit_pos = 100.0f;
float pid_integral_limit_neg = -100.0f;

// Constants
// RC Channels
const int RC_THROTTLE_CH = 2;
const int RC_ROLL_CH = 0;
const int RC_PITCH_CH = 1;
const int RC_YAW_CH = 3;
const int RC_EXTRA1_CH = 4;
const int RC_EXTRA2_CH = 5;

// IMU Registers
const int IMU_I2C_ADDRESS = 104;    // 0x68
const int IMU_REG_SMPLRT_DIV = 25;  // 0x19
const int IMU_REG_PWR_MGMT1 = 107;  // 0x6B
const int IMU_REG_WHO_AM_I = 117;   // 0x75
const int IMU_REG_CONFIG = 26;      // 0x1A
const int IMU_REG_GYRO_CONFIG = 27; // 0x1B
const int IMU_REG_ACCE_CONFIG = 28; // 0x1C

// IMU Data Registers
const int IMU_REG_ACCE_X_HI = 59;   // 0x3B
const int IMU_REG_ACCE_X_LO = 60;   // 0x3C
const int IMU_REG_ACCE_Y_HI = 61;   // 0x3D
const int IMU_REG_ACCE_Y_LO = 62;   // 0x3E
const int IMU_REG_ACCE_Z_HI = 63;   // 0x3F
const int IMU_REG_ACCE_Z_LO = 64;   // 0x40
const int IMU_REG_TEMP_HI = 65;     // 0x41
const int IMU_REG_TEMP_LO = 66;     // 0x42
const int IMU_REG_GYRO_X_HI = 67;   // 0x43
const int IMU_REG_GYRO_X_LO = 68;   // 0x44
const int IMU_REG_GYRO_Y_HI = 69;   // 0x45
const int IMU_REG_GYRO_Y_LO = 70;   // 0x46
const int IMU_REG_GYRO_Z_HI = 71;   // 0x47
const int IMU_REG_GYRO_Z_LO = 72;   // 0x48

// Gyro Index and Offsets
const int GYRO_INDEX_ROLL = 0;
const int GYRO_INDEX_PITCH = 1;
const int GYRO_INDEX_YAW = 2;
const int GYRO_X_OFFSET = 0;
const int GYRO_Y_OFFSET = 1;
const int GYRO_Z_OFFSET = 2;

void setup() {
    // acce_range, gyro_range, max_throttle_rate, and min_throttle_rate are defined above

    // Declare variables
    float SCALE_MULTIPLIER_ACCE, SCALE_MULTIPLIER_GYRO;
    float THROTTLE_RANGE;

    // Calculate values inside a function
    SCALE_MULTIPLIER_ACCE = acce_range / 32767.0f;
    SCALE_MULTIPLIER_GYRO = gyro_range / 32767.0f;
    THROTTLE_RANGE = max_throttle_rate - min_throttle_rate;

}
void main(){
    printf("Hello World \n");
    printf("%f \n", pid_kd_roll);
    printf("%d \n", gyro_range);

}
