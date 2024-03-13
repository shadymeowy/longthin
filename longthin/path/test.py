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

ps = path[0].points(path[0].length)
plt.plot(ps[:, 0], ps[:, 1], color='blue', linestyle='--')
ps = path[1].points(path[1].length)
plt.plot(ps[:, 0], ps[:, 1], color='blue', linestyle='--')
ps = path[2].points(path[2].length)
plt.plot(ps[:, 0], ps[:, 1], color='blue', linestyle='--')

line = Line(np.array([0., 1., 0.]), np.array([6., -2., 0.]))
ps = line.points(line.length)
plt.plot(ps[:, 0], ps[:, 1], color='red', linestyle='--')

for path_ in path:
    for p in path_.intersection(line):
        plt.scatter(p[0], p[1], color='green')

arc = Arc(np.array([2., 0., 0.]), np.array([-2.2, 0, 0.]),
          np.array([0., -2.2, 0.]), np.array([0., 0., -1.]))
ps = arc.points(arc.length)
plt.plot(ps[:, 0], ps[:, 1], color='red', linestyle='--')

for path_ in path:
    for p in path_.intersection(arc):
        plt.scatter(p[0], p[1], color='green')

p = np.array([0., -2., 0.])
p_c = line.closest(p)
plt.scatter(p[0], p[1], color='black')
plt.scatter(p_c[0], p_c[1], color='yellow')

plt.gca().set_aspect('equal', adjustable='box')
plt.show()
