import numpy as np
import logging

logger = logging.getLogger(__name__)


class EKF:
    def __init__(self, X, P, R, Q, delta_v, md, timeout):
        self.X = X
        self.P = P
        self.R = R
        self.Q = Q
        self.delta_v = delta_v
        self.md = md
        self.timeout = timeout
        self.vxprev = 0
        self.vyprev = 0
        self.last_update = 0

    def f(self, dt):
        px, py, vx, vy, dvelbx, dvelby, cbr, cbj = self.X
        dvelx, dvely = self.delta_v
        vxnew = -cbj*dvely + cbr*dvelx - dvelbx + vx
        vynew = cbj*dvelx + cbr*dvely - dvelby + vy
        pxnew = px + 0.5 * (vx + self.vxprev) * dt
        pynew = py + 0.5 * (vy + self.vyprev) * dt
        self.vxprev = vx
        self.vyprev = vy
        return np.array(
            [pxnew,
             pynew,
             vxnew,
             vynew,
             dvelbx,
             dvelby,
             cbr,
             cbj])

    def F(self, dt):
        px, py, vx, vy, dvelbx, dvelby, cbr, cbj = self.X
        dvelx, dvely = self.delta_v
        return np.array(
            [[1, 0, dt, 0, 0, 0, 0, 0],
             [0, 1, 0, dt, 0, 0, 0, 0],
             [0, 0, 1, 0, -1, 0, dvelx, -dvely],
             [0, 0, 0, 1, 0, -1, dvely, dvelx],
             [0, 0, 0, 0, 1, 0, 0, 0],
             [0, 0, 0, 0, 0, 1, 0, 0],
             [0, 0, 0, 0, 0, 0, 1, 0],
             [0, 0, 0, 0, 0, 0, 0, 1]])

    def h(self):
        px, py, vx, vy, dvelbx, dvelby, cbr, cbj = self.X
        return np.array([px, py, cbr, cbj])

    def H(self):
        return np.array(
            [[1, 0, 0, 0, 0, 0, 0, 0],
             [0, 1, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, 1, 0],
             [0, 0, 0, 0, 0, 0, 0, 1]])

    def predict(self, dt):
        self.last_update += dt
        if self.last_update > self.timeout:
            return self.X, self.P
        P = self.P
        X = self.f(dt)
        F = self.F(dt)
        Q = self.Q
        P = F @ P @ F.T + Q
        self.X = X
        self.P = P
        return X, P

    def update(self, Z):
        P = self.P
        X = self.X
        H = self.H()
        h = self.h()
        y = Z - h
        S = H @ P @ H.T + self.R
        md = y.T @ np.linalg.inv(S) @ y
        md = np.sqrt(md)
        if md > self.md:
            return md, False
        K = P @ H.T @ np.linalg.inv(S)
        X = X + K @ y
        n = len(X)
        P = (np.eye(n) - K @ H) @ P
        cbr, cbj = X[6], X[7]
        c = np.array([cbr, cbj])
        c /= np.linalg.norm(c)
        X[6], X[7] = c
        self.X = X
        self.P = P
        self.last_update = 0
        return md, True


class EKFAdapter:
    def __init__(self):
        self.ekf = None
        self._ev_pos = None
        self._ev_angle = None
        self._ev_flag = False
        self._imu_dvel = None
        self._imu_angle = None
        self._imu_dt = None
        self._imu_flag = False
        self._predict_flag = False

        self.P0 = np.diag([10., 10., 10., 10., 1e-3, 1e-3, 10., 10.])
        self.R = np.diag([0.07, 0.07, 1e-2, 1e-2])
        self.Q = np.diag([1e-4, 1e-4, 2e-2, 2e-2, 1e-7, 1e-7, 1e-6, 1e-6])
        self.md = 3.
        self.x = 0
        self.y = 0
        self.angle = 0

    def _init(self):
        if self._ev_pos is None or self._ev_angle is None:
            return False
        if self._imu_dvel is None or self._imu_angle is None:
            return False
        ev_pos = self._ev_pos
        bias = self._angle_bias()
        self.ekf = EKF(
            X=np.array([ev_pos[0], ev_pos[1], 0., 0., 0., 0., np.cos(bias), np.sin(bias)]),
            P=self.P0, R=self.R, Q=self.Q, md=self.md, timeout=0.5,
            delta_v=np.zeros(2))
        return True

    def set_ev_pose(self, ev_pos, ev_angle):
        x, y = ev_pos
        self._ev_pos = ev_pos
        self._ev_angle = ev_angle
        self._ev_flag = True
        if self.ekf is None:
            self._init()

    def set_imu_dvel(self, imu_dvel, imu_angle, imu_dt):
        self._imu_angle = imu_angle
        self._imu_dvel = imu_dvel
        self._imu_dt = imu_dt
        self._imu_flag = True

    def _angle_bias(self):
        return self._ev_angle - self._imu_angle

    def step(self):
        ret = 0, True
        if self.ekf is None and not self._init():
            return ret
        if self._ev_flag and self._predict_flag:
            self._ev_flag = False
            self._predict_flag = False
            bias = self._angle_bias()
            x = self._ev_pos[0]
            y = self._ev_pos[1]
            ret = self.ekf.update(np.array([x, y, np.cos(bias), np.sin(bias)]))
        if self._imu_flag:
            self._imu_flag = False
            angle_bias = np.arctan2(self.ekf.X[7], self.ekf.X[6])
            self.angle = (self._imu_angle + angle_bias) % (2 * np.pi)
            self.ekf.delta_v = self._imu_dvel
            self._predict_flag = True
            self.ekf.predict(self._imu_dt)
            self.x, self.y = self.ekf.X[:2]
        return ret
