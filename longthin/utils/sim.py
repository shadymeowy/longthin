import numpy as np
import time
import argparse
from scipy.spatial.transform import Rotation as R

from ..node import LTNode
from ..graphics import LTRenderer
from ..config import load_config
from ..ltpacket import *
from ..model import ddmr_dynamic_model, RK4
from ..shm import SHMVideoWriter


def main():
    parser = argparse.ArgumentParser(description='A simulation of the robot')
    parser.add_argument('--debug', action='store_true', help='Print debug messages')
    parser.add_argument('--video', default='shared:lt_video', help='Video sink')
    args = parser.parse_args()

    node = LTNode()
    config = node.config
    params = node.params
    renderer = LTRenderer(config, headless=True)
    cam_model = config.camera.model
    width, height = cam_model.width, cam_model.height
    if not args.video.startswith('shared:'):
        raise ValueError('Only shared memory video is supported')
    sink = args.video.strip('shared:')
    sink = SHMVideoWriter(sink, width, height)

    def cb_motor_output(packet):
        nonlocal left, right
        left, right = packet.left, packet.right

    node.subscribe(MotorOutput, cb_motor_output)

    model = ddmr_dynamic_model(**config.sim.model._asdict())
    sim_period = config.sim.dt
    cam_period = config.sim.cam_dt
    north_offset = np.deg2rad(config.sim.north_offset)
    y0 = [
        config.sim.initial.w,
        config.sim.initial.v,
        np.deg2rad(config.sim.initial.theta),
        config.sim.initial.x,
        config.sim.initial.y
    ]
    solver = RK4(model, 0, y0, sim_period)
    left, right = 0, 0
    y = None
    vel_x_prev = 0
    vel_y_prev = 0
    t_prev = time.perf_counter()

    # I did not think knowing game loop strategies would be useful some day...
    sim_accumulator = 0.0
    cam_accumulator = 0.0
    imu_accumulator = 0.0
    spin_accumulator = 0.0
    last_time = time.perf_counter()

    y = solver.step((left, right))
    while True:
        current_time = time.perf_counter()
        delta_time = current_time - last_time
        last_time = current_time

        spin_accumulator += delta_time
        sim_accumulator += delta_time
        cam_accumulator += delta_time
        imu_accumulator += delta_time

        imu_period = params.imu_publish_period / 1e6
        if imu_accumulator >= imu_period:
            imu_accumulator -= imu_period
            rot = R.from_euler('xyz', [0, 0, y[2]+north_offset], degrees=False)
            q = rot.as_quat()
            vel = y[1]
            vel_x = vel * np.cos(y[2])
            vel_y = vel * np.sin(y[2])
            dvx = vel_x - vel_x_prev
            dvy = vel_y - vel_y_prev
            dvt = current_time - t_prev
            vel_x_prev = vel_x
            vel_y_prev = vel_y
            t_prev = current_time
            dvz = 0
            packet = Imu(
                q[3], q[0], q[1], q[2],
                dvx, dvy, dvz,
                dvt)
            node.publish(packet)
            packet = SimState(
                y[0],
                y[1],
                np.rad2deg(y[2]) % 360,
                y[3],
                y[4],
                0)
            node.publish(packet)

        if sim_accumulator >= sim_period:
            sim_accumulator -= sim_period
            y = solver.step((left, right))

        if cam_accumulator >= cam_period:
            cam_accumulator -= cam_period
            renderer.vehicle_pose.pos = np.array([y[3], y[4], 0.])
            renderer.vehicle_pose.att = np.array([0., 0., np.rad2deg(y[2])])
            renderer.update_coordinates()
            img = renderer.render_image()
            sink.write(img)

        spin_period = 1e-2
        if spin_accumulator >= spin_period:
            spin_accumulator -= spin_period
            node.spin_once()

        time.sleep(1e-3)


if __name__ == "__main__":
    main()
