from sympy import *
init_printing()


def cmul(c1, c2):
    return c1[0]*c2[0] - c1[1]*c2[1], c1[0]*c2[1] + c1[1]*c2[0]


def cconj(c):
    return c[0], -c[1]


def crot(c, p):
    pc = p[0] + I*p[1]
    c = c[0] + I*c[1]
    pc_rot = pc*c
    return Matrix([re(pc_rot), im(pc_rot)])


# State vector of the kalman filter
# px, py is the position of the vehicle
px, py = symbols('px py', real=True)
p = Matrix([px, py])
# vx, vy is the velocity of the vehicle
vx, vy = symbols('vx vy', real=True)
v = Matrix([vx, vy])
# dvelbx, dvelby is the delta velocity bias of the vehicle
dvelbx, dvelby = symbols('dvelbx dvelby', real=True)
dvelbias = Matrix([dvelbx, dvelby])
# cbr, cbj is the rotation of the vehicle in xy plane
cbr, cbj = symbols('cbr cbj', real=True)
cbias = Matrix([cbr, cbj])

# Time varying variables
# dvelx, dvely is the delta velocity bias of the vehicle
dvelx, dvely = symbols('dvelx dvely', real=True)
dvel_meas = Matrix([dvelx, dvely])
# cr, cj is the rotation of the vehicle
cr, cj = symbols('cr cj', real=True)
c_meas = Matrix([cr, cj])

# Auxiliary variables
# dt is the time step
dt = symbols('dt')
sigma_dvel = symbols('sigma_dvel', real=True)
sigma_cbias = symbols('sigma_c', real=True)

# remove bias from the rotation measurement
c = cmul(cconj(cbias), c_meas)
# remove bias from the delta velocity measurement
dvel = dvel_meas - dvelbias
dvel = crot(cbias, dvel)

# The state vector
X = Matrix([p, v, dvelbias, cbias])
print('X:')
pprint(X)

# State transition function
f = Matrix([
    p + v*dt,
    v + dvel,
    dvelbias,
    cbias
])
f = simplify(f)
print('f:')
pprint(f)

# Jacobian of f
F = f.jacobian(X)
F = simplify(F)
print('F:')
pprint(F)

# Disturbance matrix
G = f.jacobian(Matrix([c_meas, dvel_meas]))
print('G:')
pprint(G)

# Disturbance vector
w = diag(sigma_cbias, sigma_cbias, sigma_dvel, sigma_dvel)
print('w:')
pprint(w)

# Process noise
Q = G*w*G.T
print('Q:')
pprint(Q)

# Measurement function of the position and rotation
h = Matrix([p, cbias])
print('h:')
pprint(h)

# Jacobian of h
H = h.jacobian(X)
print('H:')
pprint(H)

# Print the results
print('x:')
print(str(X.T))
print('f:')
print(str(f.T))
print('F:')
print(str(F))
print('Q:')
print(str(Q))
print('h:')
print(str(h.T))
print('H:')
print(str(H))