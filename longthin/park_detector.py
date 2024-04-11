import cv2
import numpy as np

from fast_shape_finder import (simple_edge, ransac_line, simple_denoise,
                               remove_inliers, calculate_lch_bounds, mask_image)


class ParkDetector:
    def __init__(
            self,
            width,
            height,
            inlier_threshold=100,
            height_offset=250,
            width_offset=0,
            slope_limit=0.5,
            hue_orange=(np.deg2rad(-5),  np.deg2rad(30)),
            hue_blue=(np.deg2rad(150),  np.deg2rad(270)),
            chrome_orange=(0.2, 1.),
            intensity_orange=(0.15, 1.),
            chroma_blue=(0.2, 1.0),
            intensity_blue=(0.1, 1.),
            storage=2048):

        self.width = width
        self.height = height
        self.inlier_threshold = inlier_threshold
        self.height_offset = height_offset
        self.slope_limit = slope_limit
        self.width_offset = width_offset

        self.mask_orange = np.zeros((height, width), dtype="uint8")
        self.mask_blue = np.zeros((height, width), dtype="uint8")
        self.pps = np.zeros((storage, 2), dtype="int32")
        self.pps_d = np.zeros((storage, 2), dtype="int32")
        self.pps_d2 = np.zeros((storage, 2), dtype="int32")
        self.inliers = np.zeros(storage, dtype="uint8")
        self.kernel1 = np.ones((5, 5), np.uint8)
        self.kernel2 = np.ones((25, 25), np.uint8)

        self.bounds_orange = calculate_lch_bounds(
            hue_orange, chrome_orange, intensity_orange)
        self.bounds_blue = calculate_lch_bounds(
            hue_blue, chroma_blue, intensity_blue)

    def process_image(self, image, debug=False):
        mask_image(image, self.bounds_blue, self.mask_blue)
        mask_image(image, self.bounds_orange, self.mask_orange)

        mask_orange = cv2.morphologyEx(self.mask_orange, cv2.MORPH_OPEN, self.kernel1)
        mask_blue = cv2.erode(self.mask_blue, self.kernel2, iterations=1)
        mask_blue = cv2.dilate(mask_blue, self.kernel2, iterations=2)
        mask_and = cv2.bitwise_and(mask_orange, mask_blue)

        if debug:
            cv2.imshow("Original", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
            cv2.imshow("Orange", mask_orange*255)
            cv2.imshow("Blue", mask_blue*255)
            cv2.imshow("And", mask_and*255)

        delta = 1
        count = 2000
        ln = simple_edge(mask_and, self.pps, 512, 256)
        ln_d = simple_denoise(self.pps[:ln], self.pps_d, 4)

        lines = []
        intercepts = []
        pps_d = self.pps_d[:ln_d]
        pps_d2 = self.pps_d2[:ln_d]
        inliers = self.inliers[:ln_d]

        k = self.slope_limit
        y1 = image.shape[0] - self.height_offset
        inlier_count = 0
        y_min = None
        if ln_d > 100:
            for _ in range(4):
                line = ransac_line(pps_d, inliers, count, delta, k)
                inliers = inliers.astype(bool)
                lines.append(line)
                # TODO: Fix this
                inliers_ps = pps_d[inliers[:pps_d.shape[0]]]
                mn = np.min(inliers_ps[:, 1])
                if y_min is None or mn < y_min:
                    y_min = mn
                inlier_count += np.sum(inliers)

                x = -(y1-line["py"])/line["nx"]*line["ny"]+line["px"]
                intercepts.append(x)

                count = remove_inliers(pps_d, pps_d2, inliers)
                pps_d, pps_d2 = pps_d2[:count], pps_d[:count]

            mean_x = sum(intercepts) / 4
            if debug:
                for line in lines:
                    image = draw_line(image, **line)
                for x in intercepts:
                    cv2.circle(image, [int(x), int(y1)], 5, [255, 0, 0], -1)
                cv2.circle(image, [int(mean_x), int(y1)], 5, [255, 0, 255], -1)
                cv2.line(image, [int(image.shape[1] / 2), 0], [int(image.shape[1] / 2), image.shape[0]], [0, 0, 255], 2)
                cv2.imshow("Result", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        else:
            mean_x = None

        if inlier_count < self.inlier_threshold:
            mean_x = None
        return mean_x, y_min


def draw_line(image, px, py, nx, ny):
    nl = np.sqrt(nx * nx + ny * ny)
    nx /= nl
    ny /= nl
    vx = ny * 1000
    vy = -nx * 1000
    return cv2.line(image, [int(px - vx), int(py - vy)], [int(px + vx), int(py + vy)], [0, 255, 0], 4)
