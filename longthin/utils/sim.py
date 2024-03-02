import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
import argparse

from ..graphics import LTRendererParams, LTRenderer
from ..ltpacket import *
from ..model import ddmr_dynamic_model, RK4


def main():
    parser = argparse.ArgumentParser(description='A simple blink example')
    parser.add_argument('--zmq', default=5555, help='ZMQ port')
    parser.add_argument('--zmq2', default=5556, help='ZMQ port2')
    parser.add_argument('--debug', action='store_true', help='Print debug messages')
    args = parser.parse_args()
    conn = LTZmq(args.zmq, args.zmq2, server=False)

    params = LTRendererParams()
    renderer = LTRenderer(params)

    model = ddmr_dynamic_model(
        R=0.0175,
        L=0.05,
        M=2.0,
        d=0.3,
        J=0.06166666666666666,
        B=3.0,
        h=0.3,
        Kb=0.06820926132509801,
        Ki=0.45,
        Rs=1.25)
    dt = 1e-3
    solver = RK4(model, 0, [0, 0, 0, 0, 0], dt)
    left, right = 0, 0
    last_time = time.time()
    y = None

    while True:
        while True:
            packet = conn.read()
            if packet is None:
                break
            if isinstance(packet, MotorOutput):
                left, right = packet.left, packet.right

        img = renderer.render_image()
        t = time.time()
        step_count = (t - last_time) / dt
        step_count = int(step_count) + 1
        last_time += step_count * dt
        sim_time = step_count * dt
        calc_time = time.time()
        for _ in range(step_count):
            y = solver.step((left, right))
        calc_time = time.time() - calc_time
        print("rate", sim_time / calc_time)
        yaw = np.rad2deg(y[2]) % 360
        vel = y[1]
        conn.send(Imu(0, 0, 360-yaw, vel))

        renderer.vehicle_pose.pos = np.array([y[3], y[4], 0.])
        renderer.vehicle_pose.att = np.array([0., 0., yaw])

        if renderer.draw():
            break


if __name__ == "__main__":
    main()
