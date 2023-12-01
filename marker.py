import numpy as np
import cv2

marker_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
marker_params = cv2.aruco.DetectorParameters()
"""marker_params.polygonalApproxAccuracyRate = 0.1
marker_params.minMarkerPerimeterRate = 0.01"""
marker_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
detector = cv2.aruco.ArucoDetector(marker_dict, marker_params)


def marker_gen(n, size=6):
    result = []
    for i in range(n):
        marker = cv2.aruco.generateImageMarker(marker_dict, i, size)
        marker = marker[1:-1, 1:-1] >> 7
        result.append(marker)
    result = np.array(result)
    return result


def marker_detect(img):
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(img_gray)
    corners = np.array(corners)
    return corners, ids


def marker_draw(img, corners, ids):
    if ids is not None:
        img = img.astype(np.float32)
        img = cv2.aruco.drawDetectedMarkers(img, corners, ids)
        img = img.astype(np.uint8)
    return img


if __name__ == '__main__':
    markers = marker_gen(8)
    for i, marker in enumerate(markers):
        cv2.imwrite(f'markers/{i}.png', marker*255)
