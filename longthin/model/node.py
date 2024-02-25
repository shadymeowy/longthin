import time
import numpy as np

from ..ltpacket import *
from .ddmr import *
from .rk4 import *


def main():
    conn = LTZmq(5555, 5556, server=False)
    model = ddmr_dynamic_model(
        R=0.0175,
        L=0.05,
        M=2.0,
        d=0.3,
        J=0.06166666666666666,
        B=1.0,
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

        t = time.time()
        step_count = (t - last_time) / dt
        step_count = int(step_count) + 1
        last_time += step_count * dt
        sim_time = step_count * dt
        calc_time = time.time()
        for _ in range(step_count):
            y = solver.step((right, left))
        calc_time = time.time() - calc_time
        print("calc time", calc_time)
        print("sim time", sim_time)
        print("rate", sim_time / calc_time)
        yaw = np.rad2deg(y[2]) % 360
        vel = y[1]
        conn.send(Imu(0, 0, yaw, vel))
        time.sleep(1/30)


if __name__ == '__main__':
    main()
