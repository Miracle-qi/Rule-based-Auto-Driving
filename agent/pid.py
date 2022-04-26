import math
import bisect
import carla
import numpy as np

import common


class PID_controller:

    def __init__(self, car_model, dt):
        self.car = car_model
        self.dt = dt
        self.Acc_Table = {0: 0, 2: 0.3, 4: 0.4, 10: 0.55, 15: 0.65, 20: 0.7, 30: 1}
        self.v_accu_err = 0
        self.v_last_err = 0
        self.yaw_accu_err = 0
        self.yaw_last_err = 0
        self.cte_accu = 0
        self.cte_last = 0


    def update(self, rx, ry, ryaw, poly_coe):
        V_p, V_i, V_d = 0.5, 0.001, 0.3
        index = bisect.bisect(list(self.Acc_Table.keys()), poly_coe[0]) - 1
        delta = poly_coe[0] - list(self.Acc_Table.keys())[index]
        slope = (list(self.Acc_Table.values())[index+1] - list(self.Acc_Table.values())[index])\
                /(list(self.Acc_Table.keys())[index + 1] - list(self.Acc_Table.keys())[index])
        feed_f = list(self.Acc_Table.values())[index] + slope * delta
        v_err = poly_coe[0] - self.car.v
        self.v_accu_err += v_err
        d_v_err = (v_err - self.v_last_err)/self.dt
        self.v_last_err = v_err
        thro = feed_f + V_p * v_err + V_d * d_v_err
        # print("feed_f", feed_f)
        # print("v_err", v_err)
        # print("v_accu_err", self.v_accu_err)
        # print("d_v_err", d_v_err)
        # print("thro", thro)
        # print("*******")

        Y_p, Y_i, Y_d = 1.0, 0.01, 0.2
        yaw_err = ryaw[20] - self.car.yaw
        if yaw_err > math.pi:
            yaw_err = yaw_err - 2 * math.pi
        elif yaw_err < -math.pi:
            yaw_err = yaw_err + 2 * math.pi
        self.yaw_accu_err += yaw_err
        d_yaw_err = (yaw_err - self.yaw_last_err)/self.dt
        self.yaw_last_err = yaw_err
        steering_from_angle = Y_p * yaw_err + Y_i * self.yaw_accu_err + Y_d * d_yaw_err
        # steer = self.car.steer + S_p * yaw_err + S_d * d_yaw_err
        # print("ryaw[0]", ryaw[0])
        # print("car.yaw", self.car.yaw)
        # print("yaw_err", yaw_err)
        # print("yaw_accu_err", self.yaw_accu_err)
        # print("d_yaw_err", d_yaw_err)
        # print("Current steer", self.car.steer)
        # print("steer", steering_from_angle)
        # print("*******")

        P_p, P_i, P_d = 0.8, 0.001, 0.3
        dis_err = np.hypot(self.car.x - rx[0], self.car.y - ry[0])
        phi = math.atan2(self.car.y - ry[0], self.car.x - rx[0])
        delta = common.pi_2_pi(ryaw[0]-phi)
        cte = math.sin(delta) * dis_err # Cross Track Error
        self.cte_accu += cte
        d_cte = (cte - self.cte_last)/self.dt
        self.cte_last = cte
        steering_from_pos = P_p * cte + P_i * self.cte_accu + P_d * d_cte

        # print("dis_err", dis_err)
        # print("phi", phi)
        # print("delta", delta)
        # print("cte", cte)
        # print("self.cte_accu", self.cte_accu)
        # print("d_cte", d_cte)
        # print("steering_from_pos", steering_from_pos)

        steering = steering_from_pos + steering_from_angle




        control = carla.VehicleControl()
        if (thro < 0):
            control.throttle = 0
            control.brake = min(abs(thro), 1)
        else:
            control.throttle = min(thro, 1)
            control.brake = 0
        control.steer = max(min(steering, 1), -1)

        return control