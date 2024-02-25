import cv2
import numpy as np

from .graphics import LTParams, LTRenderer
from .estimator import Estimator
from .geometry import intersection_plane_line


def main():
    params = LTParams()
    renderer = LTRenderer(params)
    estimator = Estimator(params)

    while not renderer.draw():
        if not params.checker_enable:
            angle = np.deg2rad(renderer.vehicle_pose.att[2])
            renderer.vehicle_pose.pos = np.array(
                [np.sin(angle), -np.cos(angle), 0.]) * 1
            renderer.vehicle_pose.att += np.array([0., 0., 0.23])

        vehicle_pose = renderer.vehicle_pose
        camera_pose = vehicle_pose.from_frame(renderer.camera_pose)
        camera_params = renderer.camera_params

        img = renderer.render_image()
        compound = estimator.estimate(img)
        if compound is None:
            continue
        est, corners, ids, actual_corners, calculated_corners = compound
        print('est', *est)
        print('act', *camera_pose.pos[:2], camera_pose.att[2])

        renderer.drawlist_area.style2(0., 0., 1., 1., 4.)
        for x, y in actual_corners:
            renderer.drawlist_area.point(x, y, 0)
        rays = camera_params.rays(camera_pose.att, corners)
        renderer.drawlist_area.style2(1., 0., 1., 0.2, 2.)
        for ray in rays:
            ray *= 4
            renderer.drawlist_area.line(camera_pose.pos[0], camera_pose.pos[1], camera_pose.pos[2],
                                        camera_pose.pos[0] + ray[0], camera_pose.pos[1] + ray[1], camera_pose.pos[2] + ray[2])
        renderer.drawlist_area.style2(1., 0., 1., 1., 4.)
        for ray in rays:
            # find intersection with ground
            pg = np.array([0., 0., params.marker_alt])
            ng = np.array([0., 0., 1.])
            p = intersection_plane_line((pg, ng), (camera_pose.pos, ray))
            renderer.drawlist_area.point(p[0], p[1], p[2])

        renderer.drawlist_area.style2(0., 1., 0., 1., 4.)
        for corner in calculated_corners:
            corner = np.array([corner[0], corner[1], 0])
            corner[:2] += renderer.camera_pose.pos[:2]
            corner = vehicle_pose.from_frame(corner)
            renderer.drawlist_area.point(corner[0], corner[1], corner[2])

        cv2.imshow("Image", img)
        cv2.waitKey(1)
