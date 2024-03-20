import cv2
import time
from pprint import pprint
from picamera2 import Picamera2
from libcamera import controls


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
            "AfMode": controls.AfModeEnum.Auto,
            "AfRange": controls.AfRangeEnum.Normal,
            "AfTrigger": controls.AfTriggerEnum.Start,
            "AeEnable": True,
            "AeExposureMode": controls.AeExposureModeEnum.Short,
            "AeConstraintMode": controls.AeConstraintModeEnum.Highlight,
        })

    def read(self):
        return True, self.pc2.capture_array()

    def print_info(self):
        print("conf")
        pprint(self.pc2.camera_configuration())
        print("prop")
        pprint(self.pc2.camera_properties)
        print("control")
        pprint(self.pc2.camera_controls)


if __name__ == "__main__":
    from .config import load_config
    config = load_config("default.yaml")
    width, height = config.camera.model.width, config.camera.model.height
    cam = PiCam(width, height)
    cam.print_info()

    while True:
        ret, img = cam.read()
        if not ret:
            print("Failed to get frame")
            break
        cv2.imshow('frame', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
