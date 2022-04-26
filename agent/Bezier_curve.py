import math
import numpy as np
import matplotlib.pyplot as plt
import scipy.special


def calc_path(sx, sy, syaw, ex, ey, eyaw, offset, d_dist):
    """
    Compute control points and path given start and end position.
    :param sx: (float) x-coordinate of the starting point
    :param sy: (float) y-coordinate of the starting point
    :param syaw: (float) yaw angle at start
    :param ex: (float) x-coordinate of the ending point
    :param ey: (float) y-coordinate of the ending point
    :param eyaw: (float) yaw angle at the end
    :param offset: (float)
    :return: (numpy array, numpy array)
    """
    length = np.hypot(sx - ex, sy - ey)
    dist = length / offset
    n_points = int(length / d_dist)
    control_points = np.array(
        [[sx, sy],
         [sx + dist * np.cos(syaw), sy + dist * np.sin(syaw)],
         [ex - dist * np.cos(eyaw), ey - dist * np.sin(eyaw)],
         [ex, ey]])

    rx, ry, ryaw = [], [], []
    for t in np.linspace(0, 1, n_points):
        p = bezier(t, control_points)
        rx.append(p[0])
        ry.append(p[1])
        ryaw.append(calc_yaw(t, control_points))
    s = calc_s(rx, ry)

    return rx, ry, ryaw, s


def bernstein_poly(n, i, t):
    """
    Bernstein polynom.
    :param n: (int) polynom degree
    :param i: (int)
    :param t: (float)
    :return: (float)
    """
    return scipy.special.comb(n, i) * t ** i * (1 - t) ** (n - i)


def bezier(t, control_points):
    """
    Return one point on the bezier curve.
    :param t: (float) number in [0, 1]
    :param control_points: (numpy array)
    :return: (numpy array) Coordinates of the point
    """
    n = len(control_points) - 1
    return np.sum([bernstein_poly(n, i, t) * control_points[i] for i in range(n + 1)], axis=0)


def bezier_derivatives_control_points(control_points, n_derivatives):
    """
    Compute control points of the successive derivatives of a given bezier curve.
    A derivative of a bezier curve is a bezier curve.
    See https://pomax.github.io/bezierinfo/#derivatives
    for detailed explanations
    :param control_points: (numpy array)
    :param n_derivatives: (int)
    e.g., n_derivatives=2 -> compute control points for first and second derivatives
    :return: ([numpy array])
    """
    w = {0: control_points}
    for i in range(n_derivatives):
        n = len(w[i])
        w[i + 1] = np.array([(n - 1) * (w[i][j + 1] - w[i][j])
                             for j in range(n - 1)])
    return w


def calc_curvature(dx, dy, ddx, ddy):
    """
    Compute curvature at one point given first and second derivatives.
    :param dx: (float) First derivative along x axis
    :param dy: (float)
    :param ddx: (float) Second derivative along x axis
    :param ddy: (float)
    :return: (float)
    """
    return (dx * ddy - dy * ddx) / (dx ** 2 + dy ** 2) ** (3 / 2)

def calc_yaw(t, control_points):
    u"""
    calc yaw
    """
    derivatives_cp = bezier_derivatives_control_points(control_points, 1)
    dt = bezier(t, derivatives_cp[1])
    yaw = math.atan2(dt[1], dt[0])
    return yaw

def calc_s(rx, ry):
    dx = np.diff(rx)
    dy = np.diff(ry)
    ds = [math.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]
    s = [0]
    s.extend(np.cumsum(ds))
    return s

if __name__ == '__main__':

    rx, ry, ryaw = calc_path(0, 0, math.pi/2, 20, 80, math.pi/2, 3, 100)

    plt.figure()
    plt.plot(rx, ry, "r")
    plt.figure()
    plt.plot(ryaw, "g")
    plt.show()