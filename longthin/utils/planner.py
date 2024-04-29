import argparse
from dataclasses import asdict

from ..node import LTNode
from ..planner import Planner
from ..ltpacket import *


def main():
    parser = argparse.ArgumentParser(description='The parking planner')
    args = parser.parse_args()

    node = LTNode()
    planner = Planner(node)
    node.spin()
