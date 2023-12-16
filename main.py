import cv2
import numpy as np

from raspicam import PiCam
from estimator import Estimator
from ltparams import LTParams


def main():
    cam = PiCam()
    params = LTParams()
    estimator = Estimator(params)
    while True:
        img = cam.capture()
        cv2.imshow("image", cv2.resize(img, (1280, 720)))
        compound = estimator.estimate(img)
        if compound is not None:
            est = compound[0]
            est[:2] *= 100
            print(est)
        cv2.waitKey(1)


if __name__ == "__main__":
    main()
