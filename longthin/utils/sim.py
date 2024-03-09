import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
import argparse

from ..graphics import LTRenderer
from ..config import load_config
from ..ltpacket import *
from ..model import ddmr_dynamic_model, RK4
from ..shm import SHMVideoWriter


def main():
    parser = argparse.ArgumentParser(description='A simulation of the robot')
    parser.add_argument('--zmq', default=5555, help='ZMQ port')
    parser.add_argument('--zmq2', default=5556, help='ZMQ port2')
    parser.add_argument('--debug', action='store_true', help='Print debug messages')
    args = parser.parse_args()
    conn = LTZmq(args.zmq, args.zmq2, server=False)

    config = load_config('default.yaml')
    renderer = LTRenderer(config)
    width, height = config.camera.model.width, config.camera.model.height
    writer = SHMVideoWriter('lt_video', width, height)

    model = ddmr_dynamic_model(**config.sim.model._asdict())
    dt = config.sim.dt
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
        if last_time + 1/30 < time.time():
            writer.write(img)
            t = time.time()
            step_count = (t - last_time) / dt
            step_count = int(step_count) + 1
            last_time += step_count * dt
            sim_time = step_count * dt
            calc_time = time.time()
            for _ in range(step_count):
                y = solver.step((left, right))
            calc_time = time.time() - calc_time
            rate = sim_time / calc_time
            yaw = np.rad2deg(y[2]) % 360
            vel = y[1]
            conn.send(Imu(0, 0, yaw, vel))

            packet = SimState(*y, rate)
            conn.send(packet)

            renderer.vehicle_pose.pos = np.array([y[3], y[4], 0.])
            renderer.vehicle_pose.att = np.array([0., 0., yaw])

        if renderer.draw():
            break


if __name__ == "__main__":
    main()
