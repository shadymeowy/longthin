from sympy import *
init_printing()

# Define the symbols
u = symbols('u[0:2]')  # input (right+left, right-left)
y = symbols('y[0:5]')  # state (w, v, theta, xp, yp)
y_dot = symbols('y_dot[0:5]')  # state derivative
R = symbols('R')  # wheel radius
L = symbols('L')  # distance between wheels
M = symbols('M')  # mass of body
d = symbols('d')  # distance from wheels to body center
J = symbols('J')  # inertia of body
B = symbols('B')  # friction coefficient of front wheel
h = symbols('h')  # distance to front wheel
Kb = symbols('Kb')  # back emf constant
Ki = symbols('Ki')  # torque constant
Rs = symbols('Rs')  # resistance of motor

# The dynamic model equations
v_inr = (u[0] + u[1]) / 2
v_inl = (u[0] - u[1]) / 2
w, v, theta, xp, yp = y
# Kinematic model
w_r = (1/R)*(v+L*w)
w_l = (1/R)*(v-L*w)
# Dynamic model of actuators
v_ar = v_inr - Kb*w_r
v_al = v_inl - Kb*w_l
tau_r = (Ki/Rs)*v_ar
tau_l = (Ki/Rs)*v_al
# Dynamic of body model
v_dot = (1/(M*R))*(tau_r+tau_l) + d*w**2 - (B/M)*v
w_dot = L/(R*(M*d**2+J))*(tau_r-tau_l) - M*d/(M*d**2+J)*v*w - B*h*(h+d)/(M*d**2+J)*w
# Position of body from velocities
theta_dot = w
# Position respect to axis not center
xp_dot = v*cos(theta)
yp_dot = v*sin(theta)

# Output the equations
poly_exprs = [w_dot, v_dot]
# To polynomial form
polys = [poly(e, [*u, *y]) for e in poly_exprs]
# Get the coefficients
coeffs = set()
for p in polys:
    monoms = p.monoms()
    for m in monoms:
        c = p.coeff_monomial(m)
        coeffs.add(c)
coeffs = list(coeffs)
# Apply cse to simplify the coefficients
tmps, consts = cse(coeffs)
# Name the constants
consts = [(Symbol(f"c{i}"), c) for i, c in enumerate(consts)]
# Output the tmps
for k, v in tmps:
    print(f"{k} = {v}")
# Output the constants
for i, c in consts:
    print(f"{i} = {c}")
# Output the equations
for p, yd in zip(polys, y_dot):
    monoms = p.monoms()
    expr = 0
    for m in monoms:
        monom = prod([s**e for s, e in zip([*u, *y], m)])
        c = p.coeff_monomial(m)
        # find the index from coeffs
        idx = coeffs.index(c)
        # find the index from consts
        c = consts[idx][0]
        expr += c*monom
    print(f"{yd} = {expr}")

# Print the non-polynomial part
non_poly_exprs = [theta_dot, xp_dot, yp_dot]
for e, yd in zip(non_poly_exprs, y_dot[len(polys):]):
    print(f"{yd} = {e}")
