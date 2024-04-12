import argparse
import cv2
import numpy as np

from ..node import LTNode
from ..ltpacket import *
from ..config import load_config
from ..estimator import Estimator
from ..park_detector import ParkDetector
from ..video_source import video_source


def main():
    parser = argparse.ArgumentParser(description='A packet echo client')
    parser.add_argument('--video', default="shared:lt_video", help='Video source')
    parser.add_argument('--show', action='store_true', help='Show camera feed')
    args = parser.parse_args()

    node = LTNode()
    config = load_config()
    width, height = config.camera.model.width, config.camera.model.height
    cap = video_source(args.video, width, height)
    estimator = Estimator.from_config(config)
    # TODO add parking parameters to config
    detector = ParkDetector(width, height, height_offset=-100, width_offset=0)

    while True:
        node.spin_once()
        ret, img = cap.read()
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        if not ret:
            print("Failed to get frame")
            break

        pose_est, _, img_markers, goal_points, img_ud = estimator.estimate(img, draw=True)
        mean_x, min_y = detector.process_image(img_ud, debug=args.show)

        if pose_est is not None:
            pos = pose_est.pos
            att = pose_est.att
            packet = EvPose(pos[0], pos[1], att[2] % 360)
            node.publish(packet)
            print(packet)

        if goal_points is not None:
            goal_area = cv2.contourArea(goal_points)
            goal_center = np.mean(goal_points, axis=0)
            print(f"Goal area: {goal_area}, center: {goal_center}")

        if mean_x is not None and min_y is not None:
            print(f"Mean x: {mean_x}, min y: {min_y}")

        if args.show and img_markers is not None:
            cv2.imshow('markers', img_markers)
            cv2.waitKey(1)
