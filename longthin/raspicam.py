import cv2
import time
from pprint import pprint
from picamera2 import Picamera2
from libcamera import controls

from .marker import *
from .geometry import Distortion


class PiCam:
    def __init__(self, width=1280, height=720):
        self.pc2 = Picamera2()

        mode = self.pc2.sensor_modes[1]
        config = self.pc2.create_preview_configuration(
            sensor={'output_size': mode['size'],
                    'bit_depth': mode['bit_depth']},
            main={'format': 'RGB888', 'size': (width, height)},
        )
        self.pc2.configure(config)
        self.pc2.start()
        self.pc2.set_controls({
            "AfMode": controls.AfModeEnum.Continuous,
            "AeEnable": True,
            "AeExposureMode": controls.AeExposureModeEnum.Short,
        })

    def capture(self):
        return self.pc2.capture_array()

    def print_info(self):
        print("conf")
        pprint(self.pc2.camera_configuration())
        print("prop")
        pprint(self.pc2.camera_properties)
        print("control")
        pprint(self.pc2.camera_controls)


if __name__ == "__main__":
    distortion = Distortion.from_file("other/calibration.txt")
    cam = PiCam(distortion.width, distortion.height)
    cam.print_info()
    marker_helper = MarkerHelper.from_type()

    while True:
        img = cam.capture()
        img = distortion.undistort(img)
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids = marker_helper.detect_opt(img_gray)
        print(corners)
        img_markers = marker_helper.draw(img, corners, ids)
        cv2.imshow("image", img_markers)
        cv2.waitKey(1)