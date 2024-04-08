import time
import argparse
import os
import cv2

from ..node import LTNode
from ..ltpacket import *
from ..config import load_config
from ..shm import SHMVideoWriter


def main():
    parser = argparse.ArgumentParser(description='A packet playback client')
    parser.add_argument('file', help='File from which to playback')
    parser.add_argument('--filter', '-f', default=None, help='Filter packets', nargs='+')
    parser.add_argument('--video', default='shared:lt_video', help='Video sink')
    parser.add_argument('--disable-video', action='store_true', help='Disable video sink stream')
    args = parser.parse_args()

    typs = []
    if args.filter is not None:
        print('Filtering packets', args.filter)
        for name in args.filter:
            typ = LTPacketType[name.upper()]
            typs.append(typ)
    else:
        print('No filter specified')
        typs = None

    node = LTNode()
    file = LTFileReader(args.file + ".lt")
    cam_model = node.config.camera.model
    width, height = cam_model.width, cam_model.height
    if not args.video.startswith('shared:'):
        raise ValueError('Only shared memory video is supported')
    sink = args.video.strip('shared:')
    sink = SHMVideoWriter(sink, width, height)

    frames = []
    if os.path.isdir(args.file):
        for f in os.listdir(args.file):
            t = float(os.path.splitext(f)[0])
            f = os.path.join(args.file, f)
            frames.append((t, f))
    frames.sort(key=lambda x: -x[0])

    t_packet, packet = file.read()
    if len(frames) > 0:
        t_frame, frame = frames.pop()
    else:
        t_frame, frame = t_packet, None

    offset_t = time.time() - min(t_packet, t_frame)
    while True:
        t = time.time() - offset_t
        remaining = 1e7
        if t >= t_packet and packet is not None:
            if typs is None or packet.type in typs:
                node.publish(packet)
            compound = file.read()
            if compound is None:
                packet = None
            else:
                t_packet, packet = compound
                remaining = min(remaining, max(0, t_packet - t))
        if t >= t_frame and frame is not None:
            img = cv2.imread(frame)
            if not args.disable_video:
                sink.write(img)
            if len(frames) == 0:
                frame = None
            else:
                t_frame, frame = frames.pop()
                remaining = min(remaining, max(0, t_frame - t))
        if packet is None and frame is None:
            break
        time.sleep(remaining)
    sink.close()


if __name__ == "__main__":
    main()
