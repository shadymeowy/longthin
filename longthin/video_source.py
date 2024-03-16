import cv2


def video_source(conn_str, width, height):
    if conn_str.startswith("shared:"):
        from .shm import SHMVideoCapture
        conn_str = conn_str[len("shared:"):]
        return SHMVideoCapture(conn_str, width, height)
    elif conn_str.startswith("cv2:"):
        conn_str = conn_str[len("cv2:"):]
        cap = cv2.VideoCapture(int(conn_str))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        return cap
    elif conn_str.startswith("picam:"):
        from .raspicam import PiCam
        conn_str = conn_str[len("picam:"):]
        return PiCam(width, height)
    else:
        raise ValueError("Unknown video source")
