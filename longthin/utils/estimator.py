import argparse
import cv2

from ..node import LTNode
from ..ltpacket import *
from ..config import load_config
from ..estimator import Estimator
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

    while True:
        node.spin_once()
        ret, img = cap.read()
        if not ret:
            print("Failed to get frame")
            break

        pose_est, _, img_markers, goal_points = estimator.estimate(img, draw=True)
        if img_markers is None:
            if args.show:
                cv2.imshow('markers', img)
                cv2.waitKey(1)
            continue

        pos = pose_est.pos
        att = pose_est.att
        packet = EvPose(pos[0], pos[1], att[2] % 360)
        node.publish(packet)
        print(packet)

        if args.show and img_markers is not None:
            for goal in goal_points:
                img_markers = cv2.circle(img_markers, tuple(map(int, goal)), 3, (0, 0, 255), -1)
            cv2.imshow('markers', img_markers)
            cv2.waitKey(1)
