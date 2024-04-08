import time
import argparse
import numpy as np
import matplotlib.pyplot as plt

from ..ltpacket import *
from ..calibration import *


def listen_packets(conn, timeout=5):
    start = time.time()
    while conn.read() is not None:
        pass
    result = []
    try:
        while time.time() - start < timeout:
            packet = conn.read()
            if packet is None:
                time.sleep(1e-4)
                continue
            if isinstance(packet, ImuRaw):
                result.append(packet)
    except KeyboardInterrupt:
        print('Interrupted')
    return result


def main():
    parser = argparse.ArgumentParser(description='A calibration tool for the robot')
    parser.add_argument('type', choices=['accel', 'gyro', 'mag'], help='Type of calibration')
    parser.add_argument('--file', default='calibration.csv', help='File to save calibration')
    parser.add_argument('--load', action='store_true', help='Load instead of live data')
    parser.add_argument('--debug', action='store_true', help='Print debug messages')
    args = parser.parse_args()
    conn = LTZmq()

    packet = Setparam(LTParamType.IMU_RAW_ENABLE.value, 1)
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
            packets = listen_packets(conn)
            data = np.array([[p.mag_x, p.mag_y, p.mag_z] for p in packets])
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
    elif args.type == 'accel':
        seq = [
            ('left', 'accel_x', 1),
            ('right', 'accel_x', -1),
            ('up', 'accel_y', 1),
            ('down', 'accel_y', -1),
            ('bottom', 'accel_z', 1),
            ('top', 'accel_z', -1),
        ]
        datas = []
        for name, field, _ in seq:
            print(f'Place the robot {name}')
            print('Press enter to start')
            input()
            packets = listen_packets(conn)
            data = np.array([getattr(p, field) for p in packets], dtype=np.float64)
            data = np.mean(data)
            print(f'{field}:', data)
            datas.append(data)

        A, b = accel_calibrate(*datas)
        print('A:', A)
        print('b:', b)
    elif args.type == 'gyro':
        raise NotImplementedError('Gyro calibration is not implemented yet')
    else:
        raise ValueError('Unknown type')


if __name__ == "__main__":
    main()
