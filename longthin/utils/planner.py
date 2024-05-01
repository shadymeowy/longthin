import argparse
from dataclasses import asdict
import time

from ..node import LTNode
from ..planner import Planner
from ..ltpacket import *
from ..notify import Notify


def main():
    parser = argparse.ArgumentParser(description='The parking planner')
    args = parser.parse_args()

    node = LTNode()
    while not isinstance(node.read(), Imu):
        pass

    planner = Planner(node)
    last_time = time.perf_counter()
    try:
        while True:
            t = time.perf_counter()
            last_time = t
            planner.step()
            node.spin_once()

            t = time.perf_counter()
            # TODO: add a new parameter to the planner to control the publish rate?
            period = node.params.motor_output_publish_period / 1e6
            dt = period - t + last_time
            time.sleep(max(0, dt))
    except:
        notify = Notify(node)
        notify.error()
        raise
