import numpy as np
import matplotlib.pyplot as plt
from .ddmr import ddmr_dynamic_model, ddmr_kinematic_model
from .rk4 import RK4

# Auxiliar constants
a, b = 10e-2, 60e-2
max_rpm = 140
max_rot = max_rpm * 2 * np.pi / 60
max_volt = 12
max_current = 0.8
max_torque = 3.6 / 100 * 10

# Model parameters
R = 1.75e-2
L = 5e-2
M = 2.  # 2.
d = 30e-2
J = 1/12 * M * (a**2 + b**2)
B = 0
h = 30e-2
Kb = max_volt / max_rot
Ki = max_torque / max_current
Rs = max_volt / max_current

print("Model parameters:")
print(f"R = {R}")
print(f"L = {L}")
print(f"M = {M}")
print(f"d = {d}")
print(f"J = {J}")
print(f"B = {B}")
print(f"h = {h}")
print(f"Kb = {Kb}")
print(f"Ki = {Ki}")
print(f"Rs = {Rs}")

model = ddmr_dynamic_model(R, L, M, d, J, B, h, Kb, Ki, Rs)
solver = RK4(model, 0, [0, 0, 0, 0, 0], 1e-3)
t = np.arange(0, 10, 1e-3)
y1 = np.array([solver.step((12, 6)) if t < 5 else solver.step((12, 12)) for i, t in enumerate(t)])
w_r1 = (1/R)*(y1[:, 1]+L*y1[:, 0])
w_l1 = (1/R)*(y1[:, 1]-L*y1[:, 0])

B = 1.
model = ddmr_dynamic_model(R, L, M, d, J, B, h, Kb, Ki, Rs)
solver = RK4(model, 0, [0, 0, 0, 0, 0], 1e-3)
y2 = np.array([solver.step((12, 6)) if t < 5 else solver.step((12, 12)) for i, t in enumerate(t)])
w_r2 = (1/R)*(y2[:, 1]+L*y2[:, 0])
w_l2 = (1/R)*(y2[:, 1]-L*y2[:, 0])

model = ddmr_kinematic_model(R, L, d)
solver = RK4(model, 0, [0, 0, 0], 1e-3)
y3 = np.array([solver.step((14.66, 14.66/2)) if t < 5 else solver.step((14.66, 14.66)) for i, t in enumerate(t)])
# np.savetxt("model_sim.csv", y, delimiter=",")
plt.figure()
plt.plot(y1[:, 3], y1[:, 4], label="B = 0")
plt.plot(y2[:, 3], y2[:, 4], label=f"B = {B}")
plt.plot(y3[:, 1], y3[:, 2], label="Kinematic")
plt.gca().set_aspect('equal', adjustable='box')
plt.legend()

plt.figure()
plt.plot(t, w_r1, label="w_r, B = 0")
plt.plot(t, w_l1, label="w_l, B = 0")
plt.plot(t, w_r2, label=f"w_r, B = {B}")
plt.plot(t, w_l2, label=f"w_l, B = {B}")
plt.legend()

plt.figure()
plt.plot(t, y1[:, 3], label="x, B = 0")
plt.plot(t, y2[:, 3], label="x, B = 0.5")
plt.plot(t, y3[:, 1], label="x, Kinematic")
plt.legend()

plt.figure()
plt.plot(t, y1[:, 4], label="y, B = 0")
plt.plot(t, y2[:, 4], label="y, B = 0.5")
plt.plot(t, y3[:, 2], label="y, Kinematic")
plt.legend()
plt.show()
