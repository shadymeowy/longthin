import numpy as np
import time
import argparse
from scipy.spatial.transform import Rotation as R

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
    parser.add_argument('--video', default='shared:lt_video', help='Video sink')
    args = parser.parse_args()
    conn = LTZmq(args.zmq, args.zmq2, server=False)

    config = load_config('default.yaml')
    renderer = LTRenderer(config)
    width, height = config.camera.model.width, config.camera.model.height
    if not args.video.startswith('shared:'):
        raise ValueError('Only shared memory video is supported')
    sink = args.video.strip('shared:')
    sink = SHMVideoWriter(sink, width, height)
    params = default_params()

    model = ddmr_dynamic_model(**config.sim.model._asdict())
    dt = config.sim.dt
    solver = RK4(model, 0, [0, 0, 0, 0, 0], dt)
    left, right = 0, 0
    last_time = time.time()
    y = None
    vel_x_prev = 0
    vel_y_prev = 0
    t_prev = time.time()

    while True:
        while True:
            packet = conn.read()
            if packet is None:
                break
            if isinstance(packet, Setparam):
                params[packet.type] = packet.value
            elif isinstance(packet, Setparami):
                params[packet.type] = packet.value
            elif isinstance(packet, Setparamu):
                params[packet.type] = packet.value
            elif isinstance(packet, MotorOutput):
                left, right = packet.left, packet.right

        period = params[LTParams.IMU_PUBLISH_PERIOD] / 1e6
        if last_time + period < time.time():
            img = renderer.render_image()
            sink.write(img)
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

            rot = R.from_euler('xyz', [0, 0, -y[2]], degrees=False)
            q = rot.as_quat()

            vel = y[1]
            vel_x = vel * np.cos(y[2])
            vel_y = vel * np.sin(y[2])
            dvx = vel_x - vel_x_prev
            dvy = vel_y - vel_y_prev
            dvt = t - t_prev
            vel_x_prev = vel_x
            vel_y_prev = vel_y
            t_prev = t
            dvz = 0
            packet = Imu(
                q[3], q[0], q[1], q[2],
                dvx, dvy, dvz,
                dvt)
            conn.send(packet)
            packet = SimState(*y, rate)
            conn.send(packet)

            renderer.vehicle_pose.pos = np.array([y[3], y[4], 0.])
            renderer.vehicle_pose.att = np.array([0., 0., np.rad2deg(y[2])])

        if renderer.draw():
            break


if __name__ == "__main__":
    main()
