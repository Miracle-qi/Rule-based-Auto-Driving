import math

import numpy as np
import carla
from agent.vehicle_model import *
import common


class RuleBased(object):

    def __init__(self, car_model, feature, dt):
        self.car = car_model
        self.feature = feature
        self.dt = dt
        self.confidence = 0
        self.accu_incentive_l = 0
        self.accu_incentive_r = 0
        self.old_decision = 0
        self.thr_incentive = 10
        self.stop_dist = 0

    def decision(self):

        if self.feature.stop_sign:
            print("Red Light: STOPPING")
            self.car.status = STATUS.STOPPING
            stop_dist = 1000
            stop_point = None
            for p in self.feature.stop_wps:
                _dist = np.hypot(p.transform.location.x - self.car.x, p.transform.location.y - self.car.y)
                if _dist < stop_dist:
                    stop_dist = _dist
                    stop_point = p
            self.stop_dist = stop_dist
            self.feature.world.debug.draw_point(stop_point.transform.location, life_time= 0.2)
            print("stopping_dist:", self.stop_dist)

        elif self.feature.is_junction:
            print("Traversing Junction")
            self.car.status = STATUS.FOLLOWING
            self.accu_incentive_l = 0
            self.accu_incentive_r = 0

        else:
            decision = None
            i_score_r, i_score_l = 0, 0
            if self.feature.start_sign:
                self.car.status = STATUS.FOLLOWING
                self.feature.start_sign = False

            if self.feature.cur_lane.lane_change == carla.LaneChange.Right:
                print("Lane Type: RIGHT")
                lead_info, fol_info = self.feature.find_cars_onlane(self.feature.cur_lane.get_right_lane())
                i_score_r, _v_r = self.check_lane_changing(lead_info, fol_info)
                if i_score_r:
                    decision = STATUS.START_LANE_CHANGE_R
                else:
                    decision = STATUS.FOLLOWING

            elif self.feature.cur_lane.lane_change == carla.LaneChange.Left:
                print("Lane Type: LEFT")
                lead_info, fol_info = self.feature.find_cars_onlane(self.feature.cur_lane.get_left_lane())
                i_score_l, _v_l = self.check_lane_changing(lead_info, fol_info)
                if i_score_l:
                    decision = STATUS.START_LANE_CHANGE_L
                else:
                    decision = STATUS.FOLLOWING

            elif self.feature.cur_lane.lane_change == carla.LaneChange.Both:
                print("Lane Type: BOTH")
                left_lead_info, left_fol_info = self.feature.find_cars_onlane(self.feature.cur_lane.get_left_lane())
                righ_lead_info, righ_fol_info = self.feature.find_cars_onlane(self.feature.cur_lane.get_right_lane())
                i_score_l, _v_l = self.check_lane_changing(left_lead_info, left_fol_info)
                i_score_r, _v_r = self.check_lane_changing(righ_lead_info, righ_fol_info)
                if i_score_l != None and i_score_r != None:
                    if (i_score_l >= i_score_r):
                        decision = STATUS.START_LANE_CHANGE_L
                    else:
                        decision = STATUS.START_LANE_CHANGE_R
                elif i_score_l != None and i_score_r == None:
                    decision = STATUS.START_LANE_CHANGE_L
                elif i_score_l == None and i_score_r != None:
                    decision = STATUS.START_LANE_CHANGE_R
                elif i_score_l == None and i_score_r == None:
                    decision = STATUS.FOLLOWING

            elif self.feature.cur_lane.lane_change == carla.LaneChange.NONE:
                print("Lane Type: NONE")
                decision = STATUS.FOLLOWING

            if decision == STATUS.FOLLOWING:
                self.accu_incentive_l = 0
                self.accu_incentive_r = 0
            elif decision == STATUS.START_LANE_CHANGE_R:
                self.accu_incentive_l = 0
                self.accu_incentive_r += i_score_r
            elif decision == STATUS.START_LANE_CHANGE_L:
                self.accu_incentive_l += i_score_l
                self.accu_incentive_r = 0

        print("accu_incentive", self.accu_incentive_r, self.accu_incentive_l)
        v_target, _a = self.car_following()
        if self.car.status == STATUS.FOLLOWING:
            if self.accu_incentive_r > self.thr_incentive:
                self.car.status = STATUS.START_LANE_CHANGE_R
            elif self.accu_incentive_l > self.thr_incentive:
                self.car.status = STATUS.START_LANE_CHANGE_L
        elif self.car.status == STATUS.STOPPING:
            v_target = min(self.car_stopping(self.car, self.stop_dist), v_target)
        else:
            v_target = v_target + 3

        return v_target


    def car_following(self):

        if (self.feature.find_lead_car() == None):
            return self.car.target_vel, 10
        else:
            [lead_x, lead_y, lead_l, v_lead] = self.feature.find_lead_car()

        v_des = self.car.target_vel
        v_cur = self.car.v
        s_cur = np.hypot(self.car.x - lead_x, self.car.y - lead_y) - lead_l / 2 - self.car.shape[0] / 2

        a_idm = self.IDM(v_cur, v_lead, v_des, s_cur)
        v_target = v_cur + 3 * a_idm * self.dt

        return v_target, a_idm

    def IDM(self, v_cur, v_lead, v_des, s_cur):
        '''
        :reference: https://traffic-simulation.de/info/info_IDM.html
        :return: acceleration
        '''
        a_max = 10
        T = 1.5
        s_0 = 3.0
        acc = 0.2
        dec = 3.0

        d_v = v_cur - v_lead
        s_star = s_0 + max(0, (v_cur * T + (v_cur * d_v) / (2 * math.sqrt(acc * dec))))
        acc = a_max * (1 - pow(v_cur / v_des, 4) - pow(s_star / s_cur, 2))
        return acc

    def check_lane_changing(self, lead_info, fol_info):
        '''
        :reference: https://traffic-simulation.de/info/info_MOBIL.html
        :return: incent_score and planned velocity
        '''
        b_safe = - 10
        c_polite = 0.5
        a_thr = 0.8
        v_des = self.car.target_vel
        v_ego = self.car.v
        v_idm, a_ego = self.car_following()
        ego_l = self.car.shape[0]

        delta_t = self.car.merge_length / v_ego
        rec_t = 0.2
        safe_flag = True

        if lead_info:
            [lead_x, lead_y, lead_l, v_lead] = lead_info
            s_ego = np.hypot(self.car.x - lead_x,
                             self.car.y - lead_y) + 0.3 * delta_t * v_lead - 2 - self.car.merge_length \
                    - lead_l / 2 - ego_l / 2
            if s_ego < 5:
                safe_flag = False
            print("s_ego:", s_ego)
            a_ego_idm = self.IDM(v_ego, v_lead, v_des, s_ego)
            print("idm info:", v_ego, v_lead, v_des, s_ego)
        else:
            a_ego_idm = self.IDM(v_ego, self.car.speed_max, v_des, 100)

        print("a_ego_idm:", a_ego_idm)

        if fol_info:
            [fol_x, fol_y, fol_l, v_fol, a_fol] = fol_info
            s_fol = np.hypot(self.car.x - fol_x, self.car.y - fol_y) - rec_t * v_fol - fol_l / 2 - ego_l / 2
            if s_fol < 5:
                safe_flag = False
            print("s_fol:", s_fol)
            a_fol_idm = self.IDM(v_fol, v_ego, v_fol, s_fol)
            print("a_fol_idm:", a_fol_idm)
            safe_score = a_fol_idm - b_safe
            incent_score = (a_ego_idm - a_ego) - c_polite * (a_fol - a_fol_idm) - a_thr
        else:
            safe_score = 1
            incent_score = (a_ego_idm - a_ego) - a_thr

        v_target = v_ego + 3 * a_ego_idm * self.dt

        print("safe score:", safe_score)
        print("incentive score:", incent_score)
        print("——————————————————————————")
        if safe_score > 0 and safe_flag is True:  # safe criterion
            if incent_score > 0:
                return incent_score, v_target  # incentive criterion
        return None, v_target

    def car_stopping(self, vehicle, s_sum):
        """
        The stopping strategy is too tricky and not elegant.  
        """
        a_comf = vehicle.acc_max / 3
        vel_buf = 6
        min_dist = vel_buf ** 2 / (2 * a_comf)

        if s_sum >= min_dist:
            vel_goal = 6
        elif 5 < s_sum < min_dist:
            vel_goal = vehicle.v - a_comf * vehicle.dt
        elif s_sum < 5:
            vel_goal = 0
        return vel_goal
