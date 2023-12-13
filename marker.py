import numpy as np
import cv2

marker_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
marker_params = cv2.aruco.DetectorParameters()
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
    corners = np.array(corners, dtype=np.float32)
    return corners, ids


def marker_draw(img, corners, ids):
    if ids is not None:
        img = img.astype(np.float32)
        img = cv2.aruco.drawDetectedMarkers(img, corners, ids)
        img = img.astype(np.uint8)
    return img


def marker_distribute(n, width, height, alt):
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
    markers = marker_gen(8)
    for i, marker in enumerate(markers):
        cv2.imwrite(f'markers/{i}.png', marker*255)
