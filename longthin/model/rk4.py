class RK4:
    def __init__(self, derivs, t0, y0, dt):
        self.derivs = derivs
        self.t = t0
        self.y = y0
        self.dt = dt

    def step(self, u):
        dt = self.dt
        k1 = self.derivs(self.t, self.y, u)
        k2 = self.derivs(self.t + 0.5*dt, self.y + 0.5*dt*k1, u)
        k3 = self.derivs(self.t + 0.5*dt, self.y + 0.5*dt*k2, u)
        k4 = self.derivs(self.t + dt, self.y + dt*k3, u)
        self.y = self.y + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)
        self.t += dt
        return self.y 
    

if __name__ == '__main__':
    import numpy as np
    import matplotlib.pyplot as plt
    
    ode = lambda t, y, u: np.array([y[1], -y[0]])
    y0 = np.array([0, 1])
    t0 = 0
    t1 = 2*np.pi
    dt = 1e-3
    t = np.arange(t0, t1, dt)
    solver = RK4(ode, t0, y0, dt)
    y = np.array([solver.step(0) for i, t in enumerate(t)])
    plt.plot(t, y[:, 0])
    plt.plot(t, y[:, 1])
    plt.show()