import argparse
import cv2

from ..ltpacket import *
from ..config import load_config
from ..estimator import Estimator
from ..video_source import video_source


def main():
    parser = argparse.ArgumentParser(description='A packet echo client')
    parser.add_argument('--config', default="default.yaml", help='Config file')
    parser.add_argument('--video', default="shared:lt_video", help='Video source')
    parser.add_argument('--show', action='store_true', help='Show camera feed')
    args = parser.parse_args()
    conn = LTZmq()

    config = load_config(args.config)
    width, height = config.camera.model.width, config.camera.model.height
    cap = video_source(args.video, width, height)
    estimator = Estimator.from_config(config)

    while True:
        packet = conn.read()
        ret, img = cap.read()
        if not ret:
            print("Failed to get frame")
            break

        compound = estimator.estimate(img, draw=True)
        if compound is None:
            if args.show:
                cv2.imshow('markers', img)
                cv2.waitKey(1)
            continue

        pose_est, _, img_markers = compound
        pos = pose_est.pos
        att = pose_est.att
        packet = EvPose(pos[0], pos[1], att[2] % 360)
        conn.send(packet)
        print(packet)

        if args.show and img_markers is not None:
            cv2.imshow('markers', img_markers)
            cv2.waitKey(1)
