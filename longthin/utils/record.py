import time
import argparse
import os
import cv2

from ..ltpacket import *
from ..video_source import video_source
from ..config import load_config


def main():
    parser = argparse.ArgumentParser(description='A packet record client')
    parser.add_argument('file', help='File to record to')
    parser.add_argument('--video', default=None, help='Video source')
    parser.add_argument('--zmq', default=5555, help='ZMQ port')
    parser.add_argument('--zmq2', default=5556, help='ZMQ port2')
    args = parser.parse_args()
    conn = LTZmq(args.zmq, args.zmq2, server=False)
    conf = load_config("default.yaml")
    file = LTFileWriter(args.file + ".lt")

    if args.video is not None:
        cap = video_source(
            args.video,
            conf.camera.model.width,
            conf.camera.model.height)
        os.makedirs(args.file, exist_ok=True)
    else:
        cap = None
    try:
        while True:
            while True:
                packet = conn.read()
                if packet is None:
                    time.sleep(1e-4)
                    break
                file.write(packet)
            if cap is not None:
                ret, img = cap.read()
                if not ret:
                    print("Failed to get frame")
                    break
                filename = f"{time.time():.6f}.jpg"
                filename = os.path.join(args.file, filename)
                cv2.imwrite(filename, img, [cv2.IMWRITE_JPEG_QUALITY, 100])
                print("Saved", filename)
    except KeyboardInterrupt:
        print("Exiting")


if __name__ == "__main__":
    main()
