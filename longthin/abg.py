import numpy as np


def alpha_beta_filter(x, alpha, beta):
    # check if a and b are valid
    if not (0 < alpha < 1):
        raise ValueError('0 < a < 1')
    if not (0 < beta <= 2):
        raise ValueError('0 < b <= 2')
    if not (0 < 4 - 2 * alpha - beta):
        raise ValueError('0 < 4 - 2 * a - b')

    v = np.zeros_like(x)

    def step(x_m, dt):
        nonlocal x, v
        xk = x + dt * v
        vk = v.copy()

        rk = x_m - xk
        x = xk + alpha * rk
        v = vk + beta * rk / dt
        return x

    return step
