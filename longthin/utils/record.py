import time
import argparse
import os
import cv2
import multiprocessing as mp

from ..node import LTNode
from ..ltpacket import *
from ..video_source import video_source
from ..config import load_config


def video_worker(args, conf):
    if args.video is None:
        return
    cap = video_source(
        args.video,
        conf.camera.model.width,
        conf.camera.model.height)
    os.makedirs(args.file, exist_ok=True)

    last_time = time.time()
    while True:
        ret, img = cap.read()
        if not ret:
            print("Failed to get frame")
            time.sleep(1e-4)
            continue
        current_time = time.time()
        if current_time - last_time < 1/args.video_freq:
            continue
        last_time = current_time
        filename = f"{time.time():.6f}.jpg"
        filename = os.path.join(args.file, filename)
        cv2.imwrite(filename, img)
        print("Saved", filename)


def main():
    parser = argparse.ArgumentParser(description='A packet record client')
    parser.add_argument('file', help='File to record to')
    parser.add_argument('--video_freq', default=20, help='Video period')
    parser.add_argument('--video', default=None, help='Video source')
    args = parser.parse_args()

    node = LTNode()
    file = LTFileWriter(args.file + ".lt")
    worker = mp.Process(target=video_worker, args=(args, node.config))
    worker.daemon = True
    worker.start()

    try:
        while True:
            packet = node.read()
            if packet is None:
                time.sleep(1e-4)
                continue
            file.write(packet)
    except KeyboardInterrupt:
        print("Exiting")
    finally:
        worker.terminate()


if __name__ == "__main__":
    main()
