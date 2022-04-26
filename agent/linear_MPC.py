"""
Linear MPC controller (X-Y frame)
author: huiming zhou
"""

import os
import sys
import math
import cvxpy
import carla
import numpy as np
import bisect
import matplotlib.pyplot as plt

# sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
#                 "/../../MotionPlanning/")
import common


class MPC:
    def __init__(self, car_model):

        self.car_model = car_model
        self.control = carla.VehicleControl()
        self.Acc_Table = {0: 0, 0.2: 0.5, 0.4: 0.8, 0.6: 0.9, 0.8: 0.95, 1: 1}
        # System config
        self.NX = 4  # state vector: z = [x, y, v, phi]
        self.NU = 2  # input vector: u = [acceleration, steer]
        self.dt = 0.06  # time step

        # MPC config
        self.T = 10  # finite time horizon length
        self.iter_max = 1
        self.du_res = 0.02
        self.Q = np.diag([1.0, 1.0, 3.0, .0])  # penalty for states
        self.Qf = np.diag([2.0, 2.0, 3.0, .0])  # penalty for end state
        self.R = np.diag([0.1, 0.1]) # penalty for inputs
        self.Rd = np.diag([0.1, 1.0]) # penalty for change of inputs

        self.a_opt = [0.0] * self.T
        self.delta_opt = [0.0] * self.T

    def update(self, rx, ry, ryaw, poly_coe):
        z_ref = self.calc_ref_trajectory_in_T_step(rx, ry, ryaw, poly_coe)
        self.linear_mpc_control(z_ref)
        self.control_comd(self.a_opt[0], self.delta_opt[0])
        print("reference", z_ref[3])
        # print("reference", z_ref[0][0], z_ref[1][0], z_ref[2][0], z_ref[3][0])
        # print("current: ", self.car_model.x, self.car_model.y, self.car_model.v,
        #       self.car_model.acc, self.car_model.yaw)
        # print("control:", self.control.throttle, self.control.brake, self.control.steer)
        return self.control

    def control_comd(self, acc_comd, delta_commd):
        self.control.steer = max(min(delta_commd/self.car_model.steer_max, 1), -1)
        index = bisect.bisect(list(self.Acc_Table.keys()), abs(acc_comd)/self.car_model.acc_max)-1

        if (acc_comd < 0):
            self.control.throttle = 0
            self.control.brake = abs(acc_comd)/self.car_model.acc_max
        else:
            self.control.throttle = list(self.Acc_Table.values())[index]
            self.control.brake = 0


    def calc_index(self, rx, ry, s_t):
        dx = np.diff(rx)
        dy = np.diff(ry)
        ds = [math.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]
        s = [0]
        s.extend(np.cumsum(ds))
        return bisect.bisect(s, s_t) - 1


    def calc_ref_trajectory_in_T_step(self, ref_x, ref_y, ref_yaw, sp_coe):
        """
        calc referent trajectory in T steps: [x, y, v, yaw]
        using the current velocity, calc the T points along the reference path
        :param node: current information
        :param ref_path: reference path: [x, y, yaw]
        :param sp: speed profile (designed speed strategy)
        :return: reference trajectory
        """
        z_ref = np.zeros((self.NX, self.T + 1))

        z_ref[0, 0] = ref_x[0]
        z_ref[1, 0] = ref_y[0]
        z_ref[2, 0] = sp_coe[0]
        z_ref[3, 0] = ref_yaw[0]

        for i in range(1, self.T + 1):
            t = i * self.dt
            v = sp_coe[0] + 2*sp_coe[1]*t + 3*sp_coe[2]*t**2 + 4*sp_coe[3]*t**3 + 5*sp_coe[4]*t**4
            s = sp_coe[0] * t + 2 * sp_coe[1] * t ** 2 + sp_coe[2] * t ** 3 + sp_coe[3] * t ** 4 + sp_coe[4] * t ** 5
            index = self.calc_index(ref_x, ref_y, s)

            z_ref[0, i] = ref_x[index]
            z_ref[1, i] = ref_y[index]
            z_ref[2, i] = v
            z_ref[3, i] = ref_yaw[index]

        return z_ref


    def linear_mpc_control(self, z_ref):
        """
        linear mpc controller
        :param z_ref: reference trajectory in T steps
        :param z0: initial state vector
        :param a_old: acceleration of T steps of last time
        :param delta_old: delta of T steps of last time
        :return: acceleration and delta strategy based on current information
        """
        x, y, yaw, v = None, None, None, None

        self.car_model.update()
        z0 = [self.car_model.x, self.car_model.y, self.car_model.v, self.car_model.yaw]

        # z_bar = self.predict_states_in_T_step(z0, a_old, delta_old, z_ref, model)
        # a_old, delta_old, x, y, yaw, v = self.solve_linear_mpc(z_ref, z_bar, z0, delta_old, model)

        for k in range(self.iter_max):
            z_bar = self.predict_states_in_T_step(z0, self.a_opt, self.delta_opt, z_ref)
            # a_rec, delta_rec = a_old[:], delta_old[:]
            self.a_opt, self.delta_opt, x, y, yaw, v = self.solve_linear_mpc(z_ref, z_bar, z0, self.delta_opt)
            self.car_model.update()

            # du_a_max = max([abs(ia - iao) for ia, iao in zip(a_old, a_rec)])
            # du_d_max = max([abs(ide - ido) for ide, ido in zip(delta_old, delta_rec)])
            # if max(du_a_max, du_d_max) < self.du_res:
            # break
        print("yaw:", yaw)
        # print("z_ref:", z_ref)
        # print("z_bar:", z_bar)
        # print("a_opt:", self.a_opt)
        # print("delta_opt:", self.delta_opt)
        return x, y, yaw, v


    def solve_linear_mpc(self, z_ref, z_bar, z0, d_bar):
        """
        solve the quadratic optimization problem using cvxpy, solver: OSQP
        :param z_ref: reference trajectory (desired trajectory: [x, y, v, yaw])
        :param z_bar: predicted states in T steps
        :param z0: initial state
        :param d_bar: delta_bar
        :return: optimal acceleration and steering strategy
        """

        z = cvxpy.Variable((self.NX, self.T + 1))
        u = cvxpy.Variable((self.NU, self.T))

        cost = 0.0
        constrains = []

        for t in range(self.T):
            cost += cvxpy.quad_form(u[:, t], self.R)
            cost += cvxpy.quad_form(z_ref[:, t] - z[:, t], self.Q)

            A, B, C = self.calc_linear_discrete_model(z_bar[2, t], z_bar[3, t], d_bar[t])

            constrains += [z[:, t + 1] == A @ z[:, t] + B @ u[:, t] + C]

            if t < self.T - 1:
                cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], self.Rd)
                constrains += [cvxpy.abs(u[1, t + 1] - u[1, t]) <= self.car_model.steer_change_max * self.dt]

        # print("ref:", z_ref)
        cost += cvxpy.quad_form(z_ref[:, self.T] - z[:, self.T], self.Qf)

        constrains += [z[:, 0] == z0]
        constrains += [z[2, :] <= self.car_model.speed_max]
        constrains += [z[2, :] >= -self.car_model.speed_max]
        constrains += [cvxpy.abs(u[0, :]) <= self.car_model.acc_max]
        constrains += [cvxpy.abs(u[1, :]) <= self.car_model.steer_max]

        prob = cvxpy.Problem(cvxpy.Minimize(cost), constrains)
        prob.solve(solver=cvxpy.OSQP)

        a, delta, x, y, yaw, v = None, None, None, None, None, None

        if prob.status == cvxpy.OPTIMAL or \
                prob.status == cvxpy.OPTIMAL_INACCURATE:
            x = z.value[0, :]
            y = z.value[1, :]
            v = z.value[2, :]
            yaw = z.value[3, :]
            a = u.value[0, :]
            delta = u.value[1, :]
        else:
            print("Cannot solve linear mpc!")

        return a, delta, x, y, yaw, v


    def predict_states_in_T_step(self, z0, a, delta, z_ref):

        z_bar = z_ref * 0.0

        for i in range(self.NX):
            z_bar[i, 0] = z0[i]

        for ai, di, i in zip(a, delta, range(1, self.T + 1)):
            self.car_model.predict(ai, di)
            z_bar[0, i] = self.car_model.x
            z_bar[1, i] = self.car_model.y
            z_bar[2, i] = self.car_model.v
            z_bar[3, i] = self.car_model.yaw

        return z_bar


    def calc_linear_discrete_model(self, v, phi, delta):
        """
        calc linear and discrete time dynamic model.
        :param v: speed: v_bar
        :param phi: angle of vehicle: phi_bar
        :param delta: steering angle: delta_bar
        :return: A, B, C
        """

        A = np.array([[1.0, 0.0, self.dt * math.cos(phi), - self.dt * v * math.sin(phi)],
                      [0.0, 1.0, self.dt * math.sin(phi), self.dt * v * math.cos(phi)],
                      [0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, self.dt * math.tan(delta) / self.car_model.wheelbase, 1.0]])

        B = np.array([[0.0, 0.0],
                      [0.0, 0.0],
                      [self.dt, 0.0],
                      [0.0, self.dt * v / (self.car_model.wheelbase * math.cos(delta) ** 2)]])

        C = np.array([self.dt * v * math.sin(phi) * phi,
                      -self.dt * v * math.cos(phi) * phi,
                      0.0,
                      -self.dt * v * delta / (self.car_model.wheelbase * math.cos(delta) ** 2)])
        return A, B, C



