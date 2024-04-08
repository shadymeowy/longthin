import time
import argparse
import os
import cv2

from ..ltpacket import *
from ..config import load_config
from ..shm import SHMVideoWriter


def main():
    parser = argparse.ArgumentParser(description='A packet playback client')
    parser.add_argument('file', help='File from which to playback')
    parser.add_argument('--video', default='shared:lt_video', help='Video sink')
    args = parser.parse_args()

    conn = LTZmq()
    conf = load_config()
    file = LTFileReader(args.file + ".lt")
    width, height = conf.camera.model.width, conf.camera.model.height
    if not args.video.startswith('shared:'):
        raise ValueError('Only shared memory video is supported')
    sink = args.video.strip('shared:')
    sink = SHMVideoWriter(sink, width, height)

    frames = []
    for f in os.listdir(args.file):
        t = float(os.path.splitext(f)[0])
        f = os.path.join(args.file, f)
        frames.append((t, f))
    frames.sort(key=lambda x: -x[0])

    t_packet, packet = file.read()
    t_frame, frame = frames.pop()
    offset_t = time.time() - min(t_packet, t_frame)
    while True:
        min_sleep = 0
        while True:
            t = time.time() - offset_t
            if t < t_packet or packet is None:
                min_sleep = min(min_sleep, t_packet - t)
                break
            conn.send(packet)
            compound = file.read()
            if compound is None:
                packet = None
                break
            t_packet, packet = compound
        while True:
            t = time.time() - offset_t
            if t < t_frame or frame is None:
                min_sleep = min(min_sleep, t_frame - t)
                break
            img = cv2.imread(frame)
            sink.write(img)
            if len(frames) == 0:
                frame = None
                break
            t_frame, frame = frames.pop()
        if packet is None and frame is None:
            break
        if min_sleep > 0:
            time.sleep(min_sleep * 0.5)

    sink.close()


if __name__ == "__main__":
    main()
