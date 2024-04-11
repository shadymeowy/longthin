import cv2
import numpy as np
import drawing3d

from .ltdrawlist import LTDrawList
from ..marker import MarkerHelper
from ..geometry import *
from ..config import load_config


class LTRenderer:
    def __init__(self, config, headless=False):
        self.config = config

        self.camera_pose = Pose(config.camera.pose.position, config.camera.pose.attitude)
        self.vehicle_pose = Pose([0, 0, 0], [0, 0, 0])
        self.dist_params = Distortion.from_params(**config.camera.model._asdict())
        self.camera_params = self.dist_params.camera_params
        self.camera_params_2 = self.dist_params.camera_params_2

        hfov = self.camera_params.hfov
        vfov = self.camera_params.vfov
        self.camera_vehicle = drawing3d.Camera()
        self.camera_vehicle.set_perspective(np.deg2rad(hfov), np.deg2rad(vfov))
        self.camera_vehicle.viewport = self.camera_params.width, self.camera_params.height

        self.drawlist_area = LTDrawList()
        self.drawlist_vehicle = LTDrawList()
        self.draw_area()
        self.draw_vehicle()

        if headless:
            return

        self.windows = dict()
        self.window_free = self.add_window(720, 720, b"Free")
        self.camera_free = self.window_free.get_camera()
        self.camera_free.set_perspective(45.0, 45.0)
        self.camera_free.distance = 3
        self.camera_free.rotation = (0.0, -np.pi/5, np.pi/4)

        self.window_top = self.add_window(720, 720, b"Top")
        self.camera_top = self.window_top.get_camera()
        self.camera_top.set_orthographic(0.2, 0.2)
        self.camera_top.position = (0., 0.0, 1.0)
        self.camera_top.rotation = (0.0, -np.pi/2, 0.0)
        self.window_top.controllable = False
        self.eventlist = drawing3d.EventList()

        self.windows[self.window_free].append(self.drawlist_area)
        self.windows[self.window_free].append(self.drawlist_vehicle)
        self.windows[self.window_top].append(self.drawlist_area)
        self.windows[self.window_top].append(self.drawlist_vehicle)

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
        self.drawlist_area.style2(0xc6/255, 0xe3/255, 0xff/255, 1., 1.)
        self.drawlist_area.clear()
        # base
        self.drawlist_area.style2(.8, .8, .8, 1., 1.)
        w = self.config.renderer.area.width
        h = self.config.renderer.area.height
        self.drawlist_area.plane(0, 0, 0, w, h, 0.1, 0.1)
        # strips
        self.drawlist_area.style2(1., .40, .01, 1., 1.)
        strip_w = self.config.renderer.strip.width
        # strips cover the whole area but don't overlap
        self.drawlist_area.plane(
            w/2+strip_w/2, 0, 0, strip_w, h + 2*strip_w, 0.1, 0.1)
        self.drawlist_area.plane(-w/2-strip_w/2, 0, 0,
                                 strip_w, h + 2*strip_w, 0.1, 0.1)
        self.drawlist_area.plane(
            0, h/2+strip_w/2, 0, w + 2*strip_w, strip_w, 0.1, 0.1)
        self.drawlist_area.plane(0, -h/2-strip_w/2, 0,
                                 w + 2*strip_w, strip_w, 0.1, 0.1)
        
        # strips surrounding the parking spot
        self.drawlist_area.style2(1., .40, .01, 1., 1.)
        spot_w = self.config.renderer.spot.width
        spot_l = self.config.renderer.spot.length
        spot_percent = self.config.renderer.spot.percent
        spot_x = w * spot_percent
        spot_y = h/2 + spot_l/2
        self.drawlist_area.plane(
            spot_x - spot_w/2 - strip_w/2, strip_w+spot_y, 0, strip_w, spot_l+strip_w, 0.1, 0.1)
        self.drawlist_area.plane(
            spot_x + spot_w/2 + strip_w/2, strip_w+spot_y, 0, strip_w, spot_l+strip_w, 0.1, 0.1)
        # parking spot
        self.drawlist_area.style2(0x3e/255, 0x5b/255, 0xff/255, 1., 1.)
        spot_w = self.config.renderer.spot.width
        spot_l = self.config.renderer.spot.length
        spot_percent = self.config.renderer.spot.percent
        spot_x = w * spot_percent
        spot_y = h/2 + spot_l/2
        self.drawlist_area.plane(
            spot_x, strip_w+spot_y, 0, spot_w, spot_l, spot_w, 0.1)
        self.drawlist_area.plane(
            spot_x, strip_w+spot_y+spot_w/2, 0, spot_w, spot_l-spot_w, spot_w, 0.1)

        marker_helper = MarkerHelper.default()
        border = self.config.renderer.marker.border
        for marker in self.config.markers:
            pos = marker.pose.position
            att = marker.pose.attitude
            # border
            size = marker.size + 2 * border
            pos_border = pos - np.array([0, 0, -border])
            pose = Pose(pos_border, att)
            data = np.ones((1, 1))
            self.drawlist_area.draw_binary_grid(pose, size, size, data)
            # marker
            pose = Pose(pos, att)
            data = marker_helper.generate(marker.id)
            self.draw_marker(pose, data, marker.size)

        self.drawlist_area.save()

    def draw_marker(self, pose, data, size):
        # draw line to ground
        self.drawlist_area.style2(0., 0., 0., 1., 8.)
        p = pose.pos
        # self.drawlist_area.line(p[0], p[1], p[2], p[0], p[1], 0)
        # add 2 rows and columns of zeros
        data = np.pad(data, 1, 'constant')
        self.drawlist_area.draw_binary_grid(pose, size, size, data)

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
        l = self.config.renderer.chassis.length
        w = self.config.renderer.chassis.width
        h = self.config.renderer.chassis.height
        o = self.config.renderer.chassis.offset
        wr = self.config.renderer.wheel.radius

        x, y, z = l/2 - o, 0, -h/2 - wr
        self.drawlist_vehicle.style2(0., 0., 1., 0.2, 1.)
        self.drawlist_vehicle.cuboid(x, y, z, l, w, h)
        self.drawlist_vehicle.style2(0., 0., 1., 1., 4.)
        self.drawlist_vehicle.cuboid(x, y, z, l, w, h, True)
        self.drawlist_vehicle.draw_axis(0, 0, 0, 0.1, 4)

        # draw wheels as cuboids
        r = self.config.renderer.wheel.radius
        w = self.config.renderer.wheel.width
        o = self.config.renderer.wheel.offset
        x, y, z = 0, -o, -r
        self.drawlist_vehicle.style2(0., 0., 0., 0.2, 1.)
        self.drawlist_vehicle.cuboid(x, y, z, 2*r, w, 2*r)
        self.drawlist_vehicle.style2(0., 0., 0., 1., 2.)
        self.drawlist_vehicle.cuboid(x, y, z, 2*r, w, 2*r, True)
        x, y, z = 0, o, -r
        self.drawlist_vehicle.style2(0., 0., 0., 0.2, 1.)
        self.drawlist_vehicle.cuboid(x, y, z, 2*r, w, 2*r)
        self.drawlist_vehicle.style2(0., 0., 0., 1., 2.)
        self.drawlist_vehicle.cuboid(x, y, z, 2*r, w, 2*r, True)

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
        img = img.reshape((self.camera_params.height, self.camera_params.width, 4))
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)
        img = cv2.resize(img, (self.camera_params.width, self.camera_params.height))
        img = self.dist_params.distort(img)
        return img


if __name__ == '__main__':
    config = load_config()
    renderer = LTRenderer(config)

    while not renderer.draw():
        angle = np.deg2rad(renderer.vehicle_pose.att[2])
        renderer.vehicle_pose.pos = np.array(
            [np.sin(angle), -np.cos(angle), 0.]) * 1
        renderer.vehicle_pose.att += np.array([0., 0., 0.23])

        img = renderer.render_image()
        cv2.imshow("Image", img)
        cv2.waitKey(1)
