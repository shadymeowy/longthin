import time
import argparse
import cv2
from dataclasses import asdict

from ..ltpacket import *
from ..config import load_config
from ..shm import SHMVideoCapture
from ..estimator import Estimator


def video_source(conn_str, width, height):
    if conn_str.startswith("shared:"):
        conn_str = conn_str[len("shared:"):]
        return SHMVideoCapture(conn_str, width, height)
    elif conn_str.startswith("cv2:"):
        conn_str = conn_str[len("cv2:"):]
        cap = cv2.VideoCapture(int(conn_str))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        return cap
    elif conn_str.startswith("picam:"):
        import raspicam
        conn_str = conn_str[len("picam:")]
        return raspicam.PiCam(width, height)
    else:
        raise ValueError("Unknown video source")


def main():
    parser = argparse.ArgumentParser(description='A packet echo client')
    parser.add_argument('--zmq', default=5555, help='ZMQ port')
    parser.add_argument('--zmq2', default=5556, help='ZMQ port2')
    parser.add_argument('--config', default="default.yaml", help='Config file')
    parser.add_argument('--video', default="shared:lt_video", help='Video source')
    args = parser.parse_args()
    conn = LTZmq(args.zmq, args.zmq2, server=False)

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
            continue

        pose_est, _, img_markers = compound
        pos = pose_est.pos
        att = pose_est.att
        packet = EvPose(pos[0], pos[1], att[2] % 360)
        conn.send(packet)

        if img_markers is not None:
            cv2.imshow('markers', img_markers)
            cv2.waitKey(1)

        print(pose_est)
        time.sleep(1e-4)
