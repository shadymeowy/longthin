import numpy as np

from .common import angle
from .shapes import Arc, Line


def plot_path(path, p1, p2,
              v1=None, v2=None, targets=None, outline=True,
              tangent=True, file=None, ax=None, legend=True,
              noax=False):
    import matplotlib.pyplot as plt
    if ax is None:
        plt.figure(figsize=(5, 5))
        plt.grid()
        ax = plt.gca()
    for obj in path:
        if isinstance(obj, Arc):
            e1 = np.cross(obj.n, obj.v1)
            if outline:
                theta = np.linspace(0, np.pi * 2, 100)
                ps = obj.p + np.outer(np.cos(theta), obj.v1) + \
                    np.outer(np.sin(theta), e1)
                ax.plot(ps[:, 0], ps[:, 1], color='#aaaaaa',
                        linestyle='--', zorder=-1)
            theta = np.linspace(0, obj.theta, 100)
            ps = obj.p + np.outer(np.cos(theta), obj.v1) + \
                np.outer(np.sin(theta), e1)
            ax.plot(ps[:, 0], ps[:, 1], color='black')
            ax.scatter(obj.p[0], obj.p[1], color='#aaaaaa', zorder=-1)
            ax.plot([obj.p[0], obj.q1[0]], [obj.p[1], obj.q1[1]],
                    color='#aaaaaa', linestyle='--', zorder=-1)
            ax.plot([obj.p[0], obj.q2[0]], [obj.p[1], obj.q2[1]],
                    color='#aaaaaa', linestyle='--', zorder=-1)
            if tangent:
                ax.plot([obj.q1[0], obj.q1[0] + obj.u1[0]], [
                        obj.q1[1], obj.q1[1] + obj.u1[1]], color='#ff00ff', linestyle='--')
                ax.plot([obj.q2[0], obj.q2[0] + obj.u2[0]], [
                        obj.q2[1], obj.q2[1] + obj.u2[1]], color='#ff00ff', linestyle='--')
        elif isinstance(obj, Line):
            ax.plot([obj.q1[0], obj.q2[0]], [
                obj.q1[1], obj.q2[1]], color='black')
        else:
            raise TypeError('Path must be a list of Arc or Line objects')
    # plot start and end points
    ax.scatter(p1[0], p1[1], color='red', label='Start point', s=100)
    ax.scatter(p2[0], p2[1], color='green', label='End point', s=100)
    # plot start and end vectors
    if v1 is not None:
        ax.plot([p1[0], p1[0] + v1[0]], [
                p1[1], p1[1] + v1[1]], color='red', label='Start direction')
    if v2 is not None:
        ax.plot([p2[0], p2[0] + v2[0]], [
                p2[1], p2[1] + v2[1]], color='green', label='End direction')
    if targets is not None:
        ax.scatter(targets[:, 0], targets[:, 1],
                   color='orange', label='Target points', s=100, zorder=9)
        for i, target in enumerate(targets):
            ax.annotate(f'Target {i+1}', (target[0], target[1]),
                        xytext=(target[0] + 0.2, target[1] + 0.2), zorder=10)
    # add custom legend entry for lines
    # get current limits
    ax.set_aspect('equal', adjustable='box')
    xmin, xmax = ax.get_xlim()
    ymin, ymax = ax.get_ylim()
    x = 1e4
    ax.plot([x, x], [x, x], color='black', label='Path')
    ax.plot([x, x], [x, x], color='#aaaaaa',
            linestyle='--', label='Arc outline')
    ax.scatter(x, x, color='black', label='Critical point')
    ax.scatter(x, x, color='#aaaaaa', label='Arc center')
    ax.plot([x, x], [x, x], color='#ff00ff',
            linestyle='--', label='Tangent vector to arc')
    # set limits back
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    if legend:
        plt.legend(bbox_to_anchor=(1.02, 1), loc="upper left")


if __name__ == '__main__':
    import matplotlib.pyplot as plt
    from .dubins import path_rsr

    p1 = np.array([0, 0, 0]).astype(float)
    p2 = np.array([6, 1.4, 0]).astype(float)
    v1 = np.array([1, -3, 0]).astype(float)
    v1 /= np.linalg.norm(v1)
    v2 = np.array([-1, 3, 0]).astype(float)
    v2 /= np.linalg.norm(v2)
    R = 1.5
    path = path_rsr(p1, p2, v1, v2, R)
    plot_path(path, p1, p2, v1, v2)
    plt.show()
