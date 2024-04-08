from ..ltpacket import *
import time
import argparse
import numpy as np
from scipy.spatial.transform import Rotation as R


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

        ed_kp = params[LTParamType.ED_KP]
        ed_ki = params[LTParamType.ED_KI]
        theta_kp = params[LTParamType.THETA_KP]
        theta_ki = params[LTParamType.THETA_KI]
        vdesired_kp = params[LTParamType.VDESIRED_KP]
        vdesired_ki = params[LTParamType.VDESIRED_KI]
        wdesired_kp = params[LTParamType.WDESIRED_KP]
        wdesired_ki = params[LTParamType.WDESIRED_KI]
        wheel_distance = params[LTParamType.WHEEL_DISTANCE]
        wheel_radius = params[LTParamType.WHEEL_RADIUS]

        e_d = desired_d - current_d
        e_theta = (desired_yaw % 360) - (current_yaw % 360)
        e_theta = (e_theta % 360)
        if e_theta > 180:
            e_theta -= 360

        e_d_sum += e_d * dt
        desired_v = ed_kp * e_d + ed_ki * e_d_sum

        e_theta_sum += e_theta * dt
        desired_w = theta_kp * e_theta + theta_ki * e_theta_sum

        e_v = desired_v - current_vel
        e_v_sum += e_v * dt

        e_w = desired_w - current_w
        e_w_sum += e_w * dt

        u_v = vdesired_kp * e_v + vdesired_ki * e_v_sum
        u_w = wdesired_kp * e_w + wdesired_ki * e_w_sum

        u_l = (u_v + wheel_distance * u_w / 2) / wheel_radius
        u_r = (u_v - wheel_distance * u_w / 2) / wheel_radius

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

    conn = LTZmq()
    params = default_params()

    manual_mode = 0
    u_l, u_r = 0, 0
    desired_d, desired_yaw = 0, 0
    current_d, current_yaw = 0, 0
    current_vel, current_w = 0, 0
    controller = make_controller(params)
    last_out = time.time()
    last_debug = time.time()
    while True:
        while True:
            packet = conn.read()
            if packet is None:
                time.sleep(1e-4)
                break
            if isinstance(packet, Setparam):
                params[LTParamType(packet.param)] = packet.value
            elif isinstance(packet, Setparami):
                params[LTParamType(packet.param)] = packet.value
            elif isinstance(packet, Setparamu):
                params[LTParamType(packet.param)] = packet.value
            elif isinstance(packet, Motor):
                u_l = packet.left
                u_r = packet.right
                manual_mode = 1
            elif isinstance(packet, MotorRaw):
                u_l = packet.left / 2048
                u_r = packet.right / 2048
                manual_mode = 1
            elif isinstance(packet, Setpoint):
                manual_mode = 0
                desired_d = packet.vel
                desired_yaw = packet.yaw
            elif isinstance(packet, Imu):
                rot = R.from_quat([packet.qx, packet.qy, packet.qz, packet.qw])
                current_yaw = rot.as_euler('xyz', degrees=True)[2]

        t = time.time()
        if not manual_mode:
            u_v, u_w, u_l, u_r, desired_vel, desired_w = controller(
                t, current_d, current_yaw, desired_d, desired_yaw, current_vel, current_w)

        out_enabled = params[LTParamType.MOTOR_OUTPUT_ENABLE]
        out_period = params[LTParamType.MOTOR_OUTPUT_PUBLISH_PERIOD] / 1e6
        debug_enabled = params[LTParamType.CONTROL_DEBUG_ENABLE]
        debug_period = params[LTParamType.CONTROL_DEBUG_PUBLISH_PERIOD] / 1e6
        if out_enabled and t - last_out > out_period:
            packet_out = MotorOutput(u_l, u_r)
            conn.send(packet_out)
            last_out = t
        if debug_enabled and t - last_debug > debug_period:
            packet_debug = ControlDebug(
                current_d,
                current_yaw%360,
                desired_d,
                desired_yaw%360,
                current_vel,
                current_w,
                desired_vel,
                desired_w,
                u_v, u_w,
                u_l, u_r)
            conn.send(packet_debug)
            last_debug = t


if __name__ == "__main__":
    main()
