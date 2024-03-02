import cv2
import numpy as np
import matplotlib.pyplot as plt
import time

from .graphics import LTParams, LTRenderer
from .estimator import Estimator


def main():
    params = LTParams()
    renderer = LTRenderer(params)
    estimator = Estimator(
        params.markers_ids,
        renderer.marker_corners,
        params.camera_params,
        params.distort_params
    )

    ts = []
    poses_ground = []
    poses_est = []

    start_time = time.time()
    while True:
        angle = np.deg2rad(renderer.vehicle_pose.att[2])
        renderer.vehicle_pose.pos = np.array(
            [np.sin(angle), -np.cos(angle), 0.]) * 1
        renderer.vehicle_pose.att += np.array([0., 0., 0.23])
        if renderer.draw():
            break

        vehicle_pose = renderer.vehicle_pose
        camera_pose = vehicle_pose.from_frame(renderer.camera_pose)

        img = renderer.render_image()
        compound = estimator.estimate(img, draw=True)
        if compound is None:
            continue

        pose_est, corners_pos, img_markers = compound
        if img_markers is not None:
            cv2.imshow('markers', img_markers)
            cv2.waitKey(1)

        ts.append(time.time() - start_time)
        poses_ground.append(camera_pose)
        poses_est.append(pose_est)

        renderer.drawlist_area.style2(0., 0., 1., 1., 4.)
        for x, y, z in corners_pos.reshape((-1, 3)):
            renderer.drawlist_area.point(x, y, z)

    x_ground = np.array([p.pos[0] for p in poses_ground])
    y_ground = np.array([p.pos[1] for p in poses_ground])
    yaw_ground = np.array([p.att[2] for p in poses_ground])
    x_est = np.array([p.pos[0] for p in poses_est])
    y_est = np.array([p.pos[1] for p in poses_est])
    yaw_est = np.array([p.att[2] for p in poses_est])
    plt.subplot(3, 2, 1)
    plt.plot(ts, x_ground, label='ground')
    plt.plot(ts, x_est, label='est')
    plt.title('x')
    plt.legend()
    plt.subplot(3, 2, 3)
    plt.plot(ts, y_ground, label='ground')
    plt.plot(ts, y_est, label='est')
    plt.title('y')
    plt.legend()
    plt.subplot(3, 2, 5)
    plt.plot(ts, x_ground - x_est, label='x')
    plt.plot(ts, y_ground - y_est, label='y')
    plt.title('xy error')
    plt.legend()
    plt.subplot(3, 2, 2)
    plt.plot(ts, yaw_ground, label='ground')
    plt.plot(ts, yaw_est, label='est')
    plt.title('yaw')
    plt.legend()
    plt.subplot(3, 2, 4)
    plt.plot(ts, yaw_ground - yaw_est)
    plt.title('yaw error')
    plt.show()
