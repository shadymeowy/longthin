import cv2
import numpy as np
from dataclasses import dataclass


@dataclass(frozen=True)
class MarkerHelper:
    marker_dict: any
    marker_params: any
    detector: any

    @staticmethod
    def from_dict(marker_dict_type):
        marker_dict = cv2.aruco.getPredefinedDictionary(marker_dict_type)
        marker_params = cv2.aruco.DetectorParameters()
        marker_params.polygonalApproxAccuracyRate = 0.1
        marker_params.minMarkerPerimeterRate = 0.01
        marker_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        detector = cv2.aruco.ArucoDetector(marker_dict, marker_params)
        return MarkerHelper(marker_dict, marker_params, detector)

    @staticmethod
    def default():
        return MarkerHelper.from_dict(cv2.aruco.DICT_4X4_100)

    def generate(self, id, size=6):
        marker = cv2.aruco.generateImageMarker(self.marker_dict, id, size)
        marker = marker[1:-1, 1:-1] >> 7
        return marker

    def detect(self, img_gray):
        corners, ids, _ = self.detector.detectMarkers(img_gray)
        if ids is None:
            return None, None
        corners = np.array(corners, dtype=np.float32)
        corners = corners.reshape((-1, 4, 2))
        ids = np.array(ids, dtype=np.int32)
        ids = ids.reshape((-1))
        return corners, ids

    def detect_opt(self, img_gray, downscale=3, threshold=1/16):
        height, width = img_gray.shape
        img_down = cv2.resize(img_gray, (width//downscale, height//downscale))
        corners, ids = self.detect(img_down)
        if ids is None:
            return None, None
        corners = corners * downscale
        area_thresh = (width * height) * threshold

        high_res_corners = []
        high_res_ids = []
        for corner, id in zip(corners, ids):
            # extract bounding box and apply detection again
            corner = corner.astype(np.int32)
            x_min = np.min(corner[:, 0])
            x_max = np.max(corner[:, 0])
            y_min = np.min(corner[:, 1])
            y_max = np.max(corner[:, 1])
            w = x_max - x_min
            h = y_max - y_min
            area = w * h
            # skip if bounding box size large enough
            if area > area_thresh:
                high_res_corners.append(corner)
                high_res_ids.append(id)
                continue
            x_min = max(0, x_min - w // 2)
            x_max = min(width, x_max + w // 2)
            y_min = max(0, y_min - h // 2)
            y_max = min(height, y_max + h // 2)
            roi = img_gray[y_min:y_max, x_min:x_max]
            if roi.shape[0] == 0 or roi.shape[1] == 0:
                continue
            roi_corners, ids_roi = self.detect(roi)
            if ids_roi is None:
                continue
            if id not in ids_roi:
                continue
            roi_corners = roi_corners[ids_roi == id][0]
            roi_corners[:, 0] += x_min
            roi_corners[:, 1] += y_min
            high_res_corners.append(roi_corners)
            high_res_ids.append(id)

        high_res_corners = np.array(high_res_corners, dtype=np.float32)
        high_res_ids = np.array(high_res_ids, dtype=np.int32)
        return high_res_corners, high_res_ids

    @staticmethod
    def draw(img, corners, ids):
        if ids is not None:
            img = img.astype(np.float32)
            corners = corners.reshape((-1, 1, 4, 2))
            ids = ids.reshape((-1, 1))
            img = cv2.aruco.drawDetectedMarkers(img, corners, ids)
            img = img.astype(np.uint8)
        return img

    @staticmethod
    def distribute(n, width, height, alt):
        # overly complicated marker distribution algorithm
        result = []
        result.append(np.array([width/2, height/2, alt, 45]))
        result.append(np.array([-width/2, height/2, alt, 135]))
        result.append(np.array([-width/2, -height/2, alt, 225]))
        result.append(np.array([width/2, -height/2, alt, 315]))
        # distribute markers in a square
        for j in range(4):
            angle = j * 90
            c, s = np.cos(np.deg2rad(angle)), np.sin(np.deg2rad(angle))
            w = abs(s) * width
            h = abs(c) * height
            x = c * width / 2
            y = s * height / 2
            # marker count for this side
            sw = w / (n + 1)
            sh = h / (n + 1)
            for i in range(n):
                pos = np.array([x + (i+1) * sw, y + (i+1) * sh, alt, angle])
                pos -= np.array([w/2, h/2, 0, 0])
                result.append(pos)
        return result


if __name__ == '__main__':
    import timeit
    img = cv2.imread('sim_distort.png')
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    marker = MarkerHelper.default()

    def detect():
        marker.detect(img_gray)
    print(timeit.timeit(detect, number=100))

    def detect_opt():
        marker.detect_opt(img_gray, 5)
    print(timeit.timeit(detect_opt, number=100))

    corners, ids = marker.detect_opt(img_gray)
    img = marker.draw(img, corners, ids)
    cv2.imshow('img', img)
    cv2.waitKey(0)
