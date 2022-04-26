import cvxpy as cp
import numpy as np
import math
import matplotlib.pyplot as plt


def speed_profile_quinticPoly(vehicle, vel_goal, acc_goal, s_sum):
    """
    generate smooth speed profile with quintic polynomial
    :param vehicle: current status of the vehicle [x,y,v,yaw]
    :param vel_goal: expected velocity at the end point
    :param acc_goal: expected acceleration at the end point
    :param s_sum: total journey
    :return: coefficients of quintic polynomial
    """
    vel_limit = 40
    acc_limit = 10.0
    vel_start = vehicle.v
    acc_start = vehicle.acc

    print("velocity planning", vel_start, acc_start)
    T = 2 * s_sum/(vel_goal+vel_start)
    seg = 5
    Q = [50, 0.]
    delta_t = T/seg

    a = cp.Variable(5)
    c_a_tem = [0, 2, 6*T, 12*T**2, 20*T**3]
    c_v, c_a = [], []
    N = len(np.arange(delta_t, T + delta_t/2, delta_t))
    for t in np.arange(N):
        c_v.append([1, 2 * t, 3 * t ** 2, 4 * t ** 3, 5 * t ** 4])
        c_a.append([0, 2, 6 * t, 12 * t ** 2, 20 * t ** 3])
    c_v = np.matrix(c_v)
    c_a = np.matrix(c_a)


    objective = cp.Minimize(Q[0] * cp.sum_squares(c_v @ a - np.full(N, vel_goal)) +
                            Q[1] * cp.sum_squares(c_a @ a))
    constraints = [a[0] == vel_start,
                   a[1] == acc_start,
                   c_v @ a >= np.zeros(N),
                   c_v @ a <= np.full(N, vel_limit)]

    prob = cp.Problem(objective, constraints)
    prob.solve(solver=cp.SCS)
    print(prob.status)
    print(a.value)

    return a.value


def speed_profile_uniform(vel_goal):
    return [vel_goal, 0, 0, 0, 0]


if __name__ == '__main__':
    vel_limit = 5.0
    vel_goal =  3.0
    vel_start = 0.0
    acc_limit = 3.0
    acc_start = -1.0
    acc_goal = 0.0
    T = 2/(vel_goal+vel_start)
    seg = 10
    Q = [5, 50]
    delta_t = T/seg

    a = cp.Variable(5)
    c_1 = [T, T ** 2, T ** 3, T ** 4, T ** 5]
    c_2 = [1, 2 * T, 3 * T ** 2, 4 * T ** 3, 5 * T ** 4]
    c_3 = [1, 0, 0, 0, 0]
    c_4 = [0, 2, 6*T, 12*T**2, 20*T**3]
    c_5 = [0, 2, 0, 0, 0]
    c_0, c_6, c_7 = [], [], []
    for t in np.arange(delta_t, T, delta_t):
        c_0.append([0, 0, 6, 24 * t, 60 * t ** 2])
        c_6.append([1, 2 * t, 3 * t ** 2, 4 * t ** 3, 5 * t ** 4])
        c_7.append([0, 2, 6 * t, 12 * t ** 2, 20 * t ** 3])
    c_0 = np.matrix(c_0)
    c_6 = np.matrix(c_6)
    c_7 = np.matrix(c_7)

    N = len(np.arange(delta_t, T, delta_t))

    objective = cp.Minimize(Q[0] * cp.sum_squares(c_6 @ a
                            - np.full(N, vel_goal)) + Q[1] * cp.square(cp.sum(a @ c_4) - acc_goal))
    constraints = [cp.sum(a @ c_3) == vel_start,
                   cp.sum(a @ c_5) == acc_start,
                   c_6 @ a >= np.zeros(N),
                   c_6 @ a <= np.full(N, vel_limit),
                   c_7 @ a <= np.full(N, acc_limit),
                   c_7 @ a >= -np.full(N, acc_limit)]

    prob = cp.Problem(objective, constraints)
    prob.solve(solver=cp.SCS)
    print(prob.status)
    print(a.value)

    s, v, acc = [], [], []
    ts = []
    for t in np.arange(0., T+0.01, 0.01):
        ts.append(t)
        s.append(a.value[0]*t + a.value[1]*t**2 + a.value[2]*t**3 + a.value[3]*t**4 + a.value[4]*t**5)
        v.append(a.value[0] + 2 * a.value[1] * t + 3 * a.value[2] * t ** 2 + 4 * a.value[3] * t ** 3 + 5 * a.value[4] * t ** 4)
        acc.append(2 * a.value[1] + 6 * a.value[2] * t + 12 * a.value[3] * t ** 2 + 20 * a.value[4] * t ** 3)
    plt.plot(ts, s, "r")
    plt.figure()
    plt.plot(ts, v, "b")
    plt.figure()
    plt.plot(ts, acc,"g")
    plt.show()
