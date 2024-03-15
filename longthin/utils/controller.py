from ..ltpacket import *
import time
import argparse
import numpy as np


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

        ed_kp = params[LTParams.ED_KP]
        ed_ki = params[LTParams.ED_KI]
        theta_kp = params[LTParams.THETA_KP]
        theta_ki = params[LTParams.THETA_KI]
        vdesired_kp = params[LTParams.VDESIRED_KP]
        vdesired_ki = params[LTParams.VDESIRED_KI]
        wdesired_kp = params[LTParams.WDESIRED_KP]
        wdesired_ki = params[LTParams.WDESIRED_KI]
        wheel_distance = params[LTParams.WHEEL_DISTANCE]
        wheel_radius = params[LTParams.WHEEL_RADIUS]

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

        u_l = (u_v - wheel_distance * u_w / 2) / wheel_radius
        u_r = (u_v + wheel_distance * u_w / 2) / wheel_radius

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
        return u_l, u_r
    return controller


def main():
    parser = argparse.ArgumentParser(description='A controller SITL')
    parser.add_argument('--zmq', default=5555, help='ZMQ port')
    parser.add_argument('--zmq2', default=5556, help='ZMQ port2')
    parser.add_argument('--debug', action='store_true', help='Print debug messages')
    args = parser.parse_args()
    conn = LTZmq(args.zmq, args.zmq2, server=False)

    params = default_params()

    manual_mode = 0
    motor_left, motor_right = 0, 0
    desired_d, desired_yaw = 0, 0
    current_d, current_yaw = 0, 0
    current_vel, current_w = 0, 0
    controller = make_controller(params)
    last_refresh = time.time()
    while True:
        while True:
            packet = conn.read()
            if packet is None:
                time.sleep(1e-4)
                break
            if isinstance(packet, Setparam):
                params[packet.type] = packet.value
            elif isinstance(packet, Setparami):
                params[packet.type] = packet.value
            elif isinstance(packet, Setparamu):
                params[packet.type] = packet.value
            elif isinstance(packet, Motor):
                motor_left = packet.left
                motor_right = packet.right
                manual_mode = 1
            elif isinstance(packet, MotorRaw):
                motor_left = packet.left / 2048
                motor_right = packet.right / 2048
                manual_mode = 1
            elif isinstance(packet, Setpoint):
                manual_mode = 0
                desired_d = packet.vel
                desired_yaw = packet.yaw
            elif isinstance(packet, Imu):
                current_d = packet.vel
                current_yaw = packet.yaw

        enabled = params[LTParams.MOTOR_OUTPUT_ENABLE]
        if not enabled:
            continue
        period = params[LTParams.MOTOR_OUTPUT_PUBLISH_PERIOD] / 1e6
        t = time.time()
        if t - last_refresh > period:
            continue
        last_refresh = t

        if not manual_mode:
            motor_left, motor_right = controller(
                t, current_d, current_yaw, desired_d, desired_yaw, current_vel, current_w)

        packet_out = MotorOutput(motor_left, motor_right)
        conn.send(packet_out)


if __name__ == "__main__":
    main()
