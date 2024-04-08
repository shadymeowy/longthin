import time
import argparse
import numpy as np
from scipy.spatial.transform import Rotation as R

from ..node import LTNode
from ..ltpacket import *


def make_controller(params):
    e_d_sum = 0
    e_theta_sum = 0
    e_v_sum = 0
    e_w_sum = 0
    last_t = 0

    def controller(t, current_d, current_yaw, desired_d, desired_yaw, current_vel, current_w):
        nonlocal e_d_sum, e_theta_sum, e_v_sum, e_w_sum, last_t
        dt = t - last_t
        last_t = t

        e_d = desired_d - current_d
        e_theta = (desired_yaw % 360) - (current_yaw % 360)
        e_theta = (e_theta % 360)
        if e_theta > 180:
            e_theta -= 360

        e_d_sum += e_d * dt
        desired_v = params.ed_kp * e_d + params.ed_ki * e_d_sum

        e_theta_sum += e_theta * dt
        desired_w = params.theta_kp * e_theta + params.theta_ki * e_theta_sum

        e_v = desired_v - current_vel
        e_v_sum += e_v * dt

        e_w = desired_w - current_w
        e_w_sum += e_w * dt

        u_v = params.vdesired_kp * e_v + params.vdesired_ki * e_v_sum
        u_w = params.wdesired_kp * e_w + params.wdesired_ki * e_w_sum

        u_l = (u_v + params.wheel_distance * u_w / 2) / params.wheel_radius
        u_r = (u_v - params.wheel_distance * u_w / 2) / params.wheel_radius

        if desired_d == 0:
            u_l = 0
            u_r = 0
        if u_l < 0:
            u_r -= u_l
            u_l = 0
        elif u_r < 0:
            u_l -= u_r
            u_r = 0
        u_l = np.clip(u_l, 0, 1)
        u_r = np.clip(u_r, 0, 1)
        return u_v, u_w, u_l, u_r, desired_v, desired_w
    return controller


def main():
    parser = argparse.ArgumentParser(description='A controller SITL')
    args = parser.parse_args()

    manual_mode = 0
    u_l, u_r = 0, 0
    desired_d, desired_yaw = 0, 0
    current_d, current_yaw = 0, 0
    current_vel, current_w = 0, 0
    node = LTNode()
    params = node.params
    controller = make_controller(params)

    def cb_motor(packet):
        nonlocal u_l, u_r, manual_mode
        u_l = packet.left
        u_r = packet.right
        manual_mode = 1

    def cb_motor_raw(packet):
        nonlocal u_l, u_r, manual_mode
        u_l = packet.left / 2048
        u_r = packet.right / 2048
        manual_mode = 1

    def cb_setpoint(packet):
        nonlocal desired_d, desired_yaw, manual_mode
        desired_d = packet.vel
        desired_yaw = packet.yaw
        manual_mode = 0

    def cb_imu(packet):
        nonlocal current_d, current_yaw
        rot = R.from_quat([packet.qx, packet.qy, packet.qz, packet.qw])
        current_yaw = rot.as_euler('xyz', degrees=True)[2]

    node.subscribe(Motor, cb_motor)
    node.subscribe(MotorRaw, cb_motor_raw)
    node.subscribe(Setpoint, cb_setpoint)
    node.subscribe(Imu, cb_imu)

    last_out = time.time()
    last_debug = time.time()
    while True:
        t = time.time()
        if not manual_mode:
            u_v, u_w, u_l, u_r, desired_vel, desired_w = controller(
                t, current_d, current_yaw, desired_d, desired_yaw, current_vel, current_w)

        out_period = params.motor_output_publish_period / 1e6
        debug_period = params.control_debug_publish_period / 1e6

        if t - last_out > out_period:
            last_out = t
            if params.motor_output_enable:
                packet_out = MotorOutput(u_l, u_r)
                node.publish(packet_out)
        if t - last_debug > debug_period:
            last_debug = t
            if params.control_debug_enable:
                packet_debug = ControlDebug(
                    current_d,
                    current_yaw % 360,
                    desired_d,
                    desired_yaw % 360,
                    current_vel,
                    current_w,
                    desired_vel,
                    desired_w,
                    u_v, u_w,
                    u_l, u_r)
                node.publish(packet_debug)

        node.spin_once()
        t = time.time()
        dt_out = out_period - t + last_out
        dt_debug = debug_period - t + last_debug
        dt = max(0, min(dt_out, dt_debug))
        time.sleep(dt)


if __name__ == "__main__":
    main()
