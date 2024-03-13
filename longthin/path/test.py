import matplotlib.pyplot as plt
import numpy as np

from .dubins import path_rsl
from .intersection import *
from .shapes import *
from .common import *


p1 = np.array([0, 0, 0]).astype(float)
p2 = np.array([6, 0, 0]).astype(float)
v1 = np.array([-1, 3, 0]).astype(float)
v1 /= np.linalg.norm(v1)
v2 = np.array([-1, 3, 0]).astype(float)
v2 /= np.linalg.norm(v2)
n = np.array([0, 0, 1]).astype(float)
R = 1.5
path = path_rsl(p1, p2, v1, v2, R, n)

ps = path.points()
plt.plot(ps[:, 0], ps[:, 1], color='blue')

p = np.array([2., 0., 0.])
p_c, _ = path.closest(p)
t = path.param(p_c) + 0.5
p_c2 = path.point(t)
plt.scatter(p[0], p[1], color='black')
plt.scatter(p_c[0], p_c[1], color='yellow')
plt.scatter(p_c2[0], p_c2[1], color='orange')

w, h = 4, 4
box = Path(
    Line(np.array([-w, -h, 0.]), np.array([w, -h, 0.])),
    Line(np.array([w, -h, 0.]), np.array([w, h, 0.])),
    Line(np.array([w, h, 0.]), np.array([-w, h, 0.])),
    Line(np.array([-w, h, 0.]), np.array([-w, -h, 0.]))
)
ps = box.points()
plt.plot(ps[:, 0], ps[:, 1], color='red', linestyle='--')

vps = np.array([
    [-0.5, 0.5, 0.],
    [-0.5, -0.5, 0.],
    [1.0, 0.5, 0.],
    [1.0, -0.5, 0.]
])

for path_ in path.paths_of(vps):
    ps = path_.points()
    plt.plot(ps[:, 0], ps[:, 1], color='gray')
    intr = box.intersection(path_)
    for p in intr:
        plt.scatter(p[0], p[1], color='red')

t = 7
point = path.point(t)
tangent = path.tangent(t)
plt.scatter(point[0], point[1], color='black')
plt.plot([point[0], point[0] + tangent[0]], [point[1], point[1] + tangent[1]], color='black')

for vp in vps:
    c1 = np.dot(vp, EX)
    c2 = np.dot(vp, np.cross(EZ, EX))
    vp_ = point + c1*tangent + c2*np.cross(EZ, tangent)
    plt.scatter(vp_[0], vp_[1], color='gray')


ps = box.intersection(path)
for p in ps:
    plt.scatter(p[0], p[1], color='red')
plt.gca().set_aspect('equal', adjustable='box')
plt.show()
