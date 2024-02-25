import cv2
import numpy as np

from .graphics import LTParams, LTRenderer
from .estimator import Estimator


def main():
    params = LTParams()
    renderer = LTRenderer(params)
    estimator = Estimator(params)
    
    while True:
        renderer.draw()
        vehicle_pose = renderer.vehicle_pose
        camera_pose = vehicle_pose.from_frame(renderer.camera_pose)

        img = renderer.render_image()
        compound = estimator.estimate(img)
        if compound is None:
            continue
        pose_est, corners, ids, corners_pos = compound
        print('act', camera_pose)
        print('est', pose_est)

        renderer.drawlist_area.style2(0., 0., 1., 1., 4.)
        for x, y, z in corners_pos.reshape((-1, 3)):
            renderer.drawlist_area.point(x, y, z)

        angle = np.deg2rad(renderer.vehicle_pose.att[2])
        renderer.vehicle_pose.pos = np.array(
            [np.sin(angle), -np.cos(angle), 0.]) * 1
        renderer.vehicle_pose.att += np.array([0., 0., 0.23])
        cv2.waitKey(1)