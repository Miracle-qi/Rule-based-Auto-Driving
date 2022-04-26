from agent.cubic_spline import Spline2D
from agent.Bezier_curve import calc_path
import common
import numpy as np
import math
from agent.vehicle_model import STATUS

class path_planner():

    def __init__(self, env, car_model, fea_extract):
        self.env = env
        self.car = car_model
        self.fea_extract = fea_extract
        self.d_dist = 0.1
        self.merge_dist = car_model.merge_length
        self.merge_point = None
        self.target_lane = None
        self.junction_wps = None

    def update(self):
        self.merge_dist = self.car.merge_length

        if self.car.status == STATUS.FOLLOWING:
            print("Current status: following")
            follow_lane = self.fea_extract.local_wp(self.fea_extract.wp_list[0])
            if self.fea_extract.is_junction:
                follow_lane = self.junction_waypoints(self.fea_extract.junction_lane)
            rx, ry, ryaw, s_sum = self.following_path(follow_lane)

        elif self.car.status == STATUS.STOPPING:
            print("Current status: stopping")
            follow_lane = self.fea_extract.local_wp(self.fea_extract.wp_list[0])
            rx, ry, ryaw, s_sum = self.following_path(follow_lane)

        elif self.car.status == STATUS.START_LANE_CHANGE_L:
            print("Current status: Start Left changing")
            left_lane = self.fea_extract.wp_list[0].get_left_lane()
            self.merge_point = self.merge_point_calcu(left_lane, self.merge_dist)
            self.target_lane = self.env.world.get_map().get_waypoint(self.merge_point)
            rx, ry, ryaw, s_sum = self.laneChange_path(self.car, self.target_lane, self.merge_point)
            self.car.status = STATUS.LANE_CHANGING_L

        elif self.car.status == STATUS.LANE_CHANGING_L:
            print("Current status: Left changing")
            rx, ry, ryaw, s_sum = self.laneChange_path(self.car, self.target_lane, self.merge_point)

        elif self.car.status == STATUS.START_LANE_CHANGE_R:
            print("Current status: Start Right changing")
            right_lane = self.fea_extract.wp_list[0].get_right_lane()
            self.merge_point = self.merge_point_calcu(right_lane, self.merge_dist)
            self.target_lane = self.env.world.get_map().get_waypoint(self.merge_point)
            rx, ry, ryaw, s_sum = self.laneChange_path(self.car, self.target_lane, self.merge_point)
            self.car.status = STATUS.LANE_CHANGING_R

        elif self.car.status == STATUS.LANE_CHANGING_R:
            print("Current status: Right changing")
            rx, ry, ryaw, s_sum = self.laneChange_path(self.car, self.target_lane, self.merge_point)

        return rx, ry, ryaw, s_sum


    def following_path(self, waypoints):
        wp_x, wp_y = [], []
        for point in waypoints:
            wp_x.append(point.transform.location.x)
            wp_y.append(point.transform.location.y)

        cubicspline = Spline2D(wp_x, wp_y)
        s = np.arange(0, cubicspline.s[-1], self.d_dist)

        rx, ry, ryaw, rk = [], [], [], []
        for i_s in s:
            ix, iy = cubicspline.calc_position(i_s)
            rx.append(ix)
            ry.append(iy)
            ryaw.append(common.pi_2_pi(cubicspline.calc_yaw(i_s)))
            rk.append(cubicspline.calc_curvature(i_s))

        return rx, ry, ryaw, cubicspline.s[-1]

    def laneChange_path(self, car, lane_target, merge_point):
        # generate reference line
        t_x = merge_point.x
        t_y = merge_point.y
        t_dx = lane_target.next(0.1)[0].transform.location.x-t_x
        t_dy = lane_target.next(0.1)[0].transform.location.y-t_y
        t_yaw = math.atan2(t_dy, t_dx)

        rx, ry, ryaw, s = calc_path(car.x, car.y, car.yaw, t_x, t_y, t_yaw, 3, self.d_dist)

        waypoints = common.ref_waypoint(lane_target)
        wp_x, wp_y = [], []
        for point in waypoints:
            wp_x.append(point.transform.location.x)
            wp_y.append(point.transform.location.y)
        cubicspline = Spline2D(wp_x, wp_y)

        s = np.arange(0, cubicspline.s[-1], self.d_dist)
        for i_s in s:
            ix, iy = cubicspline.calc_position(i_s)
            rx.append(ix)
            ry.append(iy)
            ryaw.append(common.pi_2_pi(cubicspline.calc_yaw(i_s)))

        if (np.hypot(car.x-t_x, car.y-t_y) < 1.0):
            car.status = STATUS.FOLLOWING

        return rx, ry, ryaw, s[-1]

    def merge_point_calcu(self, lane_target, merge_dist):
        return lane_target.next(merge_dist)[0].transform.location

    def junction_waypoints(self, fixed_lane):
        self.junction_wps = fixed_lane.next_until_lane_end(1.0)
        next_lane = self.fea_extract.map.get_waypoint(self.junction_wps[-1].transform.location)
        self.junction_wps.extend(self.extra_wp(next_lane))

        new_point, nearest_dist, index = None, 1000, 0
        for i in range(len(self.junction_wps)):
            pos = self.junction_wps[i].transform.location
            _dist = np.hypot(pos.x-self.car.x, pos.y-self.car.y)
            if _dist < nearest_dist:
                nearest_dist = _dist
                new_point = pos
                index = i

        return self.junction_wps[index:]

    def stopping_path(self, waypoints, vehicle, stop_point):
        wp_x, wp_y = [], []
        for point in waypoints:
            wp_x.append(point.transform.location.x)
            wp_y.append(point.transform.location.y)

        cubicspline = Spline2D(wp_x, wp_y)
        stop_dist = np.hypot(vehicle.x - stop_point.x, vehicle.y - stop_point.y)
        s = np.arange(0, stop_dist, self.d_dist)

        rx, ry, ryaw, rk = [], [], [], []
        for i_s in s:
            ix, iy = cubicspline.calc_position(i_s)
            rx.append(ix)
            ry.append(iy)
            ryaw.append(common.pi_2_pi(cubicspline.calc_yaw(i_s)))
            rk.append(cubicspline.calc_curvature(i_s))

        return rx, ry, ryaw, stop_dist

    def extra_wp(self, wp, max_distance=30):
        seq = 1.0
        wp_l = [wp.next(0.01)[0]]
        while True:
            wp_l.append(wp.next(seq)[0])
            seq += 1
            if seq > max_distance:
                break
        return wp_l

