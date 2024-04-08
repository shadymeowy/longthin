import argparse
import time

from ..node import LTNode
from ..ltpacket import *


def main():
    parser = argparse.ArgumentParser(description='A parameter utility client')
    parser.add_argument('param_name', type=str, help='Parameter name', nargs='?')
    parser.add_argument('--set', type=str, help='Parameter value to set')
    parser.add_argument('--default', action='store_true', help='Print default value of parameter')
    parser.add_argument('--type', action='store_true', help='Print type of parameter')
    parser.add_argument('--debug', action='store_true', help='Print debug info')
    args = parser.parse_args()

    node = LTNode()
    params = node.params

    if args.param_name is None:
        print('Available parameters:')
        print('name', 'index', 'default')
        for param, value in params.to_dict().items():
            print(hex(param.value), param.name, value)
        return
    param_name = args.param_name.upper()
    param = LTParamType[param_name]
    if args.debug:
        print('param:', param)

    if args.set is None and not args.default and not args.type:
        raise ValueError('No action specified')

    if args.set is not None:
        try:
            value = int(args.set)
        except ValueError:
            value = float(args.set)
        packet = setparam(param, value)
        if args.debug:
            print('packet:', packet)
            print('asbytes:', packet.to_bytes())
        while node.read() is None:
            time.sleep(1e-4)
        node.publish(packet)

    if args.default:
        value = default_param(param)
        print(value)


if __name__ == "__main__":
    main()
