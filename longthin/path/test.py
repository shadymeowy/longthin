import matplotlib.pyplot as plt
import numpy as np

from .dubins import path_rsl
from .intersection import *
from .shapes import Arc, Line


p1 = np.array([0, 0, 0]).astype(float)
p2 = np.array([6, 0, 0]).astype(float)
v1 = np.array([-1, 3, 0]).astype(float)
v1 /= np.linalg.norm(v1)
v2 = np.array([-1, 3, 0]).astype(float)
v2 /= np.linalg.norm(v2)
n = np.array([0, 0, 1]).astype(float)
R = 1.5
path = path_rsl(p1, p2, v1, v2, R, -n)

ps = path[0].points()
plt.plot(ps[:, 0], ps[:, 1], color='blue')
ps = path[1].points()
plt.plot(ps[:, 0], ps[:, 1], color='blue')
ps = path[2].points()
plt.plot(ps[:, 0], ps[:, 1], color='blue')
ps = path.points(uniform=True)
plt.scatter(ps[:, 0], ps[:, 1], color='black', s=3)

line = Line(np.array([0.5, 1., 0.]), np.array([6., -2., 0.]))
ps = line.points()
plt.plot(ps[:, 0], ps[:, 1], color='red', linestyle='--')

for path_ in path:
    for p in path_.intersection(line):
        plt.scatter(p[0], p[1], color='green')

arc = Arc(np.array([2., 0., 0.]), np.array([-2.2, 0, 0.]),
          np.array([0., -2.2, 0.]), np.array([0., 0., -1.]))
ps = arc.points()
plt.plot(ps[:, 0], ps[:, 1], color='red', linestyle='--')

for path_ in path:
    for p in path_.intersection(arc):
        plt.scatter(p[0], p[1], color='green')

p = np.array([2., 0., 0.])
p_c, _ = path.closest(p)
t = path.param(p_c) + 0.7
p_c2 = path.point(t)
print(p, p_c, p_c2, t)
plt.scatter(p[0], p[1], color='black')
plt.scatter(p_c[0], p_c[1], color='yellow')
plt.scatter(p_c2[0], p_c2[1], color='orange')

plt.gca().set_aspect('equal', adjustable='box')
plt.show()
