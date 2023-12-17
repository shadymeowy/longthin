import cv2
import numpy as np

from .raspicam import PiCam
from .estimator import Estimator
from .ltparams import LTParams


def main():
    params = LTParams(
        distort_enable=True,
        distort_path='other/calibration.txt',
        homography_calib_enable=True,
        homography_calib_path='other/hcalib.txt',
        homography_calibration=False)

    cam = PiCam()
    estimator = Estimator(params)
    while True:
        img = cam.capture()
        cv2.imshow("image", cv2.resize(img, (1280, 720)))
        compound = estimator.estimate(img)
        if compound is not None:
            est = compound[0]
            est[:2] *= 100
            print('est', est)
        cv2.waitKey(1)


if __name__ == "__main__":
    main()
