import numpy as np


def ddmr_dynamic_model(R, L, M, d, J, B, h, Kb, Ki, Rs):
    x0 = R**2
    x1 = 2*Kb*Ki
    x2 = Rs*x0
    x3 = B*x2
    x4 = 1/(M*Rs)
    x5 = R*Rs
    x6 = M*d**2
    c0 = x4*(-x1 - x3)/x0
    c1 = d
    c2 = Ki*L/(J*x5 + x5*x6)
    c3 = Ki*x4/R
    c4 = (-L**2*x1 - d*h*x3 - h**2*x3)/(J*x2 + x2*x6)
    c5 = -M*d/(J + x6)

    def _model(t, y, u=(0, 0)):
        u = [u[0] + u[1], u[0] - u[1]]
        y_dot = np.zeros(5)
        y_dot[0] = c2*u[1] + c4*y[0] + c5*y[0]*y[1]
        y_dot[1] = c0*y[1] + c1*y[0]**2 + c3*u[0]
        y_dot[2] = y[0]
        y_dot[3] = y[1]*np.cos(y[2])
        y_dot[4] = y[1]*np.sin(y[2])
        return y_dot
    return _model


def ddmr_kinematic_model(R, L, d):
    c0 = R/2
    c1 = R/(2*L)

    def _model(t, y, u=(0, 0)):
        u = [u[0] + u[1], u[0] - u[1]]
        y_dot = np.zeros(3)
        y_dot[0] = c1*u[1]
        y_dot[1] = c0*u[0]*np.cos(y[0])
        y_dot[2] = c0*u[0]*np.sin(y[0])
        return y_dot
    return _model
