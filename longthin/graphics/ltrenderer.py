import cv2
import numpy as np
import drawing3d

from .ltdrawlist import LTDrawList
from .ltparams import LTRendererParams
from ..marker import MarkerHelper
from ..geometry import *


class LTRenderer:
    def __init__(self, params: LTRendererParams):
        self.params = params

        self.camera_pose = Pose(params.camera_pos_rel, params.camera_att_rel)
        self.vehicle_pose = Pose(params.vehicle_pos, params.vehicle_att)
        self.camera_params = params.camera_params

        self.windows = dict()
        self.window_free = self.add_window(720, 720, b"Free")
        self.camera_free = self.window_free.get_camera()
        self.camera_free.set_perspective(45.0, 45.0)
        self.camera_free.distance = 3
        self.camera_free.rotation = (0.0, -np.pi/5, np.pi/4)
        hfov = self.params.camera_hfov
        vfov = self.params.camera_vfov
        self.camera_vehicle = drawing3d.Camera()
        self.camera_vehicle.set_perspective(np.deg2rad(hfov), np.deg2rad(vfov))
        self.camera_vehicle.viewport = self.params.camera_width, self.params.camera_height
        self.window_top = self.add_window(720, 720, b"Top")
        self.camera_top = self.window_top.get_camera()
        self.camera_top.set_orthographic(0.2, 0.2)
        self.camera_top.position = (0., 0.0, 1.0)
        self.camera_top.rotation = (0.0, -np.pi/2, 0.0)
        self.window_top.controllable = False
        self.eventlist = drawing3d.EventList()

        self.drawlist_area = LTDrawList()
        self.drawlist_vehicle = LTDrawList()
        self.windows[self.window_free].append(self.drawlist_area)
        self.windows[self.window_free].append(self.drawlist_vehicle)
        self.windows[self.window_top].append(self.drawlist_area)
        self.windows[self.window_top].append(self.drawlist_vehicle)

        self.marker_corners = None
        self.draw_area()
        self.draw_vehicle()

    def add_window(self, *args, **kwargs):
        window = drawing3d.Window(*args, **kwargs)
        self.windows[window] = list()
        return window

    def draw(self):
        self.eventlist.reset()
        self.eventlist.poll()
        self.update_coordinates()
        for window, drawlists in self.windows.items():
            if window.handle_events(self.eventlist):
                return True
            window.clear()
            for drawlist in drawlists:
                # temporary hack
                if drawlist == self.drawlist_vehicle:
                    pos = self.vehicle_pose.pos
                    att = np.deg2rad(self.vehicle_pose.att)
                    window.render_at(drawlist, pos, att)
                else:
                    window.render(drawlist)
            window.render_end()
        self.drawlist_vehicle.load()
        self.drawlist_area.load()
        return False

    def draw_area(self):
        # background
        self.drawlist_area.style2(0xa2/255, 0xbf/255, 0xf4/255, 1., 1.)
        self.drawlist_area.clear()
        # base
        self.drawlist_area.style2(.8, .8, .8, 1., 1.)
        w = self.params.area_w
        h = self.params.area_h
        self.drawlist_area.plane(0, 0, 0, w, h, 0.1, 0.1)
        # strips
        self.drawlist_area.style2(1., .2, .2, 1., 1.)
        strip_w = self.params.strip_w
        # strips cover the whole area but don't overlap
        self.drawlist_area.plane(
            w/2+strip_w/2, 0, 0, strip_w, h + 2*strip_w, 0.1, 0.1)
        self.drawlist_area.plane(-w/2-strip_w/2, 0, 0,
                                 strip_w, h + 2*strip_w, 0.1, 0.1)
        self.drawlist_area.plane(
            0, h/2+strip_w/2, 0, w + 2*strip_w, strip_w, 0.1, 0.1)
        self.drawlist_area.plane(0, -h/2-strip_w/2, 0,
                                 w + 2*strip_w, strip_w, 0.1, 0.1)
        # parking spot
        self.drawlist_area.style2(1., 1., .4, 1., 1.)
        spot_w = self.params.spot_w
        spot_l = self.params.spot_l
        spot_percent = self.params.spot_percent
        spot_x = w * spot_percent
        spot_y = h/2 + spot_l/2
        self.drawlist_area.plane(
            spot_x, strip_w+spot_y, 0, spot_w, spot_l, spot_w, 0.1)
        self.drawlist_area.plane(
            spot_x, strip_w+spot_y+spot_w/2, 0, spot_w, spot_l-spot_w, spot_w, 0.1)

        # markers
        marker_helper = MarkerHelper.default()
        marker_ids = self.params.markers_ids
        marker_datas = marker_helper.generate(marker_ids)
        marker_poses = self.params.markers_poses

        w = self.params.marker_w
        h = self.params.marker_h
        corners_local = np.array([
            [0, -w/2, -h],
            [0, w/2, -h],
            [0, w/2, 0],
            [0, -w/2, 0]
        ])

        self.marker_corners = []
        for pose, data, id in zip(marker_poses, marker_datas, marker_ids):
            pos = pose[:3]
            yaw = pose[3]
            pose = Pose(pos, [0., self.params.marker_pitch, yaw])
            corners = pose.from_frame(corners_local)
            self.marker_corners.append(corners)
            self.draw_marker(pose, data)
        self.marker_corners = np.array(self.marker_corners)

        self.drawlist_area.save()

        if self.params.checker_enable:
            self.draw_checkerboard()
            self.drawlist_area.save()

    def draw_checkerboard(self):
        self.drawlist_area.style2(0., 1., 0., 1., 4.)
        # calibration checkerboard
        checker_nw = self.params.checker_nw
        checker_nh = self.params.checker_nh
        checker_pitch = self.params.checker_pitch
        pose = Pose(np.array([
                    self.params.camera_pos_rel[0] + self.params.checker_offset,
                    self.params.camera_pos_rel[1],
                    self.params.checker_alt]),
                    np.array([0., checker_pitch, 0.]))
        pose = pose.from_frame(self.vehicle_pose)
        data = np.zeros((checker_nh, checker_nw), dtype=np.uint8)
        data[::2, ::2] = 1
        data[1::2, 1::2] = 1
        # borders
        checker_w = self.params.checker_size * checker_nw
        checker_h = self.params.checker_size * checker_nh
        self.drawlist_area.draw_binary_grid(
            pose, checker_h, checker_w, data)

    def draw_marker(self, pose, data):
        marker_w = self.params.marker_w
        marker_h = self.params.marker_h
        # draw line to ground
        self.drawlist_area.style2(0., 0., 0., 1., 8.)
        p = pose.pos
        # self.drawlist_area.line(p[0], p[1], p[2], p[0], p[1], 0)
        # add 2 rows and columns of zeros
        data = np.pad(data, 1, 'constant')
        self.drawlist_area.draw_binary_grid(pose, marker_w, marker_h, data)

    def update_coordinates(self):
        camera_pose = self.vehicle_pose.from_frame(self.camera_pose)
        self.camera_vehicle.position = camera_pose.pos
        self.camera_vehicle.rotation = np.deg2rad(camera_pose.att)

    def draw_vehicle(self):
        self.update_coordinates()
        # draw camera view first
        self.drawlist_vehicle.draw_camera_field(self.camera_pose,
                                                self.camera_params)

        # draw chassis
        l = self.params.chassis_l
        w = self.params.chassis_w
        h = self.params.chassis_h
        x, y, z = self.params.chassis_pos_rel
        self.drawlist_vehicle.style2(0., 0., 1., 0.2, 1.)
        self.drawlist_vehicle.cuboid(x, y, z, l, w, h)
        self.drawlist_vehicle.style2(0., 0., 1., 1., 4.)
        self.drawlist_vehicle.cuboid(x, y, z, l, w, h, True)
        self.drawlist_vehicle.draw_axis(0, 0, 0, 0.1, 4)

        # draw wheels as cuboids
        r = self.params.wheel_r
        w = self.params.wheel_w
        o = self.params.wheel_offset
        x, y, z = 0, -o, 0
        self.drawlist_vehicle.style2(0., 0., 0., 0.2, 1.)
        self.drawlist_vehicle.cuboid(x, y, z, r, w, r)
        self.drawlist_vehicle.style2(0., 0., 0., 1., 2.)
        self.drawlist_vehicle.cuboid(x, y, z, r, w, r, True)
        x, y, z = 0, o, 0
        self.drawlist_vehicle.style2(0., 0., 0., 0.2, 1.)
        self.drawlist_vehicle.cuboid(x, y, z, r, w, r)
        self.drawlist_vehicle.style2(0., 0., 0., 1., 2.)
        self.drawlist_vehicle.cuboid(x, y, z, r, w, r, True)

        # draw camera plane
        x, y, z = self.camera_pose.pos
        self.drawlist_vehicle.style2(1., 0., 0., 0.2, 1.)
        self.drawlist_vehicle.cuboid(x, y, z, 1e-2, 1e-2, 1e-2)
        self.drawlist_vehicle.style2(1., 0., 0., 1., 2.)
        self.drawlist_vehicle.cuboid(x, y, z, 1e-2, 1e-2, 1e-2, True)

        self.drawlist_vehicle.style2(1., 0., 0., 1., 4.)
        # line from top of chassis to camera
        self.drawlist_vehicle.line(x, y, - h, x, y, z)
        self.drawlist_vehicle.draw_camera(self.camera_pose, self.camera_params)
        self.drawlist_vehicle.save()

    def render_image(self):
        img = drawing3d.DrawList.save_buffer(self.drawlist_area, self.camera_vehicle)
        img = img.reshape((self.params.camera_height, self.params.camera_width, 4))
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)
        if self.params.distort_enable:
            img = self.params.distort_params.distort(img)
        return img


if __name__ == '__main__':
    params = LTRendererParams()
    renderer = LTRenderer(params)

    while not renderer.draw():
        angle = np.deg2rad(renderer.vehicle_pose.att[2])
        renderer.vehicle_pose.pos = np.array(
            [np.sin(angle), -np.cos(angle), 0.]) * 1
        renderer.vehicle_pose.att += np.array([0., 0., 0.23])

        img = renderer.render_image()
        cv2.imshow("Image", img)
        cv2.waitKey(1)
