#include <stdint.h>
#include <stdbool.h>

struct ltpacket_reserved_t {
    uint8_t reserved;
};

struct ltpacket_imu_t {
    float qw;
    float qx;
    float qy;
    float qz;
    float dvx;
    float dvy;
    float dvz;
    float dt;
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
    float value;
};

struct ltpacket_led_t {
    uint8_t index;
    uint8_t state;
};

struct ltpacket_control_debug_t {
    float current_yaw;
    float desired_d;
    float desired_yaw;
    float u_v;
    float u_w;
    float u_r;
    float u_l;
};

struct ltpacket_setparamu_t {
    int32_t param;
    uint32_t value;
};

struct ltpacket_setparami_t {
    int32_t param;
    int32_t value;
};

struct ltpacket_motor_output_t {
    float left;
    float right;
};

struct ltpacket_reboot_t {
    uint8_t reserved;
};

struct ltpacket_ev_pose_t {
    float x;
    float y;
    float yaw;
};

struct ltpacket_sim_state_t {
    float w;
    float v;
    float yaw;
    float x;
    float y;
    float rate;
};

struct ltpacket_ekf_state_t {
    float x;
    float y;
    float yaw;
};

struct ltpacket_setpoint_pos_t {
    float x;
    float y;
};

struct ltpacket_goal_vision_t {
    float area;
    float center_x;
    float center_y;
};

struct ltpacket_lane_vision_t {
    float mean_x;
    float min_y;
};

struct ltpacket_button_state_t {
    uint8_t index;
    uint8_t state;
};

struct ltpacket_ekf_reset_t {
    uint8_t reserved;
};

struct ltpacket_led_control_t {
    uint8_t id;
    uint8_t default_state;
    uint32_t high_time;
    uint32_t low_time;
    int32_t remaining_cycles;
};

struct ltpacket_adc_read_t {
    uint8_t id;
    float value;
};
