from ..ltpacket import *
import argparse
import yaml
import time
from dataclasses import asdict


def main():
    parser = argparse.ArgumentParser(description='A parameter utility client')
    parser.add_argument('param_name', type=str, help='Parameter name')
    parser.add_argument('--set', type=float, help='Parameter value to set')
    parser.add_argument('--default', action='store_true', help='Print default value of parameter')
    parser.add_argument('--type', action='store_true', help='Print type of parameter')
    parser.add_argument('--debug', action='store_true', help='Print debug info')
    parser.add_argument('--zmq', default=5555, help='ZMQ port')
    parser.add_argument('--zmq2', default=5556, help='ZMQ port2')
    args = parser.parse_args()

    param_name = args.param_name.upper()
    param = LTParams[param_name]
    if args.debug:
        print('param:', param)

    if args.set is None and not args.default and not args.type:
        raise ValueError('No action specified')

    if args.set is not None:
        packet = setparam_to_packet(param, args.set)
        if args.debug:
            print('packet:', packet)
            print('asbytes:', packet.to_bytes())
            print('decoded:', packet_to_setparam(packet))
        conn = LTZmq(args.zmq, args.zmq2, server=False)
        while conn.read() is None:
            time.sleep(1e-4)
        conn.send(packet)

    if args.default:
        value = default_param(param)
        print(value)


if __name__ == "__main__":
    main()
