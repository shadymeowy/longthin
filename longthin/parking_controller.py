import numpy as np


class ParkingController:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.mean_x = width / 2
        self.active = True

    def control(self):
        error = self.mean_x - self.width / 2
        sgn = np.sign(error)
        error = np.abs(error)
        error /= self.width / 2
        error **= 0.7
        error *= 1.
        error *= sgn
        error = np.clip(error, -0.3, 0.3)
        left = 0.7 + error
        right = 0.7 - error
        left = np.clip(left, 0, 0.7)
        right = np.clip(right, 0, 0.7)

        if not self.active:
            return 0, 0
        return left, right

    def update(self, mean_x, min_y):
        if mean_x is not None:
            self.mean_x = mean_x
        if min_y is not None and min_y/self.height >= 0.83:
            self.active = False
        if min_y is not None and min_y/self.height < 0.83:
            self.active = True
