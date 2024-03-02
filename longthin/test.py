import cv2
import numpy as np

from .raspicam import PiCam
from .estimator import Estimator
from .graphics import LTRendererParams


def main():
    params = LTRendererParams(
        distort_enable=True,
        distort_path='other/calibration.txt')
    params.markers = np.array([[0, 0, params.marker_alt, 0]])

    cam = PiCam(params.camera_width, params.camera_height)
    estimator = Estimator(params)
    while True:
        img = cam.capture()
        cv2.imshow("image", cv2.resize(img, (1280, 720)))
        compound = estimator.estimate(img)
        if compound is not None:
            est = compound[0]
            est[:2] *= 100
            print('est', est)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        if key == ord(' '):
            print('Writing log')
            with open('/tmp/log.txt', 'a') as f:
                f.write(' '.join(map(str, est)))
                f.write('\n')


if __name__ == "__main__":
    main()
