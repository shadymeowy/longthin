#include <stdint.h>
#include <stdbool.h>

struct ltpacket_reserved_t {
    uint8_t reserved;
};

struct ltpacket_imu_t {
    float roll;
    float pitch;
    float yaw;
    float vel;
};

struct ltpacket_imu_raw_t {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float mag_x;
    float mag_y;
    float mag_z;
};

struct ltpacket_motor_t {
    float left;
    float right;
};

struct ltpacket_motor_raw_t {
    int16_t left;
    int16_t right;
};

struct ltpacket_setpoint_t {
    float vel;
    float yaw;
};

struct ltpacket_setparam_t {
    int32_t param;
    uint32_t value;
};

struct ltpacket_led_t {
    uint8_t index;
    uint8_t state;
};

struct ltpacket_control_debug_t {
    float current_d;
    float current_yaw;
    float desired_d;
    float desired_yaw;
    float current_vel;
    float current_w;
    float desired_vel;
    float desired_w;
    float u_v;
    float u_w;
    float u_r;
    float u_l;
};
