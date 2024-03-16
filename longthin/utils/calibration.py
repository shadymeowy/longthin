import time
import argparse
import numpy as np
import matplotlib.pyplot as plt

from ..ltpacket import *
from ..calibration import *


def main():
    parser = argparse.ArgumentParser(description='A calibration tool for the robot')
    parser.add_argument('type', choices=['accel', 'gyro', 'mag'], help='Type of calibration')
    parser.add_argument('--file', default='calibration.csv', help='File to save calibration')
    parser.add_argument('--load', action='store_true', help='Load instead of live data')
    parser.add_argument('--zmq', default=5555, help='ZMQ port')
    parser.add_argument('--zmq2', default=5556, help='ZMQ port2')
    parser.add_argument('--debug', action='store_true', help='Print debug messages')
    args = parser.parse_args()
    conn = LTZmq(args.zmq, args.zmq2, server=False)

    packet = Setparam(LTParams.IMU_RAW_ENABLE.value, 1)
    while conn.read() is None:
        conn.send(packet)
        time.sleep(1e-4)

    data = []
    if args.type == 'mag':
        if args.load:
            data = np.loadtxt(args.file, delimiter=',')
        else:
            print('Rotate the robot around all axes')
            print('Press Ctrl+C to stop')
            try:
                while True:
                    packet = conn.read()
                    if packet is None:
                        time.sleep(1e-4)
                        continue
                    if isinstance(packet, ImuRaw):
                        data.append([packet.mag_x, packet.mag_y, packet.mag_z])
            except KeyboardInterrupt:
                pass
            data = np.array(data, dtype=np.float64)
            np.savetxt(args.file, data, delimiter=',')
        A, b, result = mag_calibrate(data)
        print('A:', A)
        print('b:', b)

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(data[:, 0], data[:, 1], data[:, 2], label='Data')
        ax.scatter(result[:, 0], result[:, 1], result[:, 2], label='Calibrated data')
        ax.legend()
        ax.set_box_aspect([u - l for l, u in (getattr(ax, f'get_{a}lim')() for a in 'xyz')])
        plt.show()
    else:
        raise NotImplementedError('Only magnetometer calibration is supported')


if __name__ == "__main__":
    main()
