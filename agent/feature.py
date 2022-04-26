import carla
import common
import math
import numpy as np
import copy


class FeatureExt():

    def __init__(self, env, vehicle):
        self.traffic_light_flag = None
        self.world = env.world
        self.vehicle = vehicle
        self.map = self.world.get_map()
        self.zombie_cars = env.zombie_cars
        self.cur_lane = None
        self.current_wp = None
        self.transform = None
        self.wp_list = None
        self.current_loc = None
        self.max_distance = 70
        self.distance_rate = 1.4
        self.visible_zombie_cars = []
        self.show_dt = env.dt * 1.5
        self.is_junction = False
        self.junction_lane_flag = False
        self.junction_lane = None
        self.stop_sign = False
        self.start_sign = True
        self.stop_wps = None
        self.time = 200
        self.traffic_light = None
        self.traffic_light_flag = False

    def update(self):

        self.transform = self.vehicle.get_transform()
        self.current_loc = self.transform.location
        self.cur_lane = self.map.get_waypoint(self.current_loc)
        self.is_junction = self.cur_lane.is_junction
        self.current_wp = self.local_wp(self.cur_lane)
        self.wp_list = self.wp_list_extract(self.cur_lane)

        if self.is_junction:
            if not self.junction_lane_flag:
                self.junction_lane_flag = True
                self.junction_lane = self.cur_lane
        else:
            if self.junction_lane_flag:
                print("aus")
            self.junction_lane_flag = False
        self.traffic_light_rule()

    def traffic_light_rule(self):
        # Check the state of traffic light
        if not self.traffic_light_flag:
            self.traffic_light = self.world.get_traffic_lights_from_waypoint(self.current_wp[0], 50)
        if self.traffic_light:
            self.traffic_light_flag = True
            if self.traffic_light[0].get_state() == carla.TrafficLightState.Red:
                self.stop_sign = True
                self.start_sign = False
                self.stop_wps = self.traffic_light[0].get_stop_waypoints()
            elif self.traffic_light[0].get_state() == carla.TrafficLightState.Green:
                print("Green Light")
                self.stop_sign = False
                self.start_sign = True
                self.traffic_light_flag = False
        else:
            self.stop_sign = False

        # Set the green light forcefully
        if self.stop_sign:
            self.time -= 1
            print("self.time", self.time)
            if self.time < 0:
                self.time = 200
                self.traffic_light[0].set_state(carla.TrafficLightState.Green)

    def local_wp(self, wp, max_distance=50):
        seq = 1.0
        wp_l = [wp.next(0.01)[0]]
        while True:
            wp_l.append(wp.next(seq)[0])
            seq *= self.distance_rate
            if seq > max_distance:
                break
        return wp_l

    def wp_list_extract(self, cur_wp):
        wp_l = [cur_wp]
        if cur_wp.is_junction is False:
            wp_l = self.wp_side_extract(wp_l, cur_wp, 'right')
            wp_l = self.wp_side_extract(wp_l, cur_wp, 'left')
        return wp_l

    def find_lead_car(self):
        forward_cars = []
        ego_pos = [self.vehicle.get_location().x, self.vehicle.get_location().y]
        ego_dir = self.vehicle.get_physics_control().wheels[0].position - \
                  self.vehicle.get_physics_control().wheels[2].position
        ego_vec = [ego_dir.x, ego_dir.y]
        cur_road = self.wp_list[0]

        for i in range(len(self.zombie_cars)):
            if self.zombie_cars[i] == self.vehicle:
                continue
            z_pos = self.zombie_cars[i].get_location()
            vec_car = [z_pos.x - ego_pos[0], z_pos.y - ego_pos[1]]
            dis = np.hypot(vec_car[0], vec_car[1])
            if dis < 50:
                theta = common.cal_angle(vec_car, ego_vec)
                if theta < math.pi/2 and self.check_onroad(self.zombie_cars[i], cur_road):
                    forward_cars.append([dis, i])

        if forward_cars:
            forward_cars.sort()
            lead_car = self.zombie_cars[forward_cars[0][1]]
            pos_x = lead_car.get_transform().location.x
            pos_y = lead_car.get_transform().location.y
            # self.bbox_display(lead_car, "red")
            length = lead_car.bounding_box.extent.x
            vel = np.hypot(lead_car.get_velocity().x, lead_car.get_velocity().y)
            return [pos_x, pos_y, length, vel]
        else:
                return None

    def find_cars_onlane(self, lane):
        ''' find the forwards and backwards car on the selected lane'''
        forwards_cars, backwards_cars = [], []
        ego_pos = [self.vehicle.get_location().x, self.vehicle.get_location().y]
        ego_dir = self.vehicle.get_physics_control().wheels[0].position - \
                  self.vehicle.get_physics_control().wheels[2].position
        ego_vec = [ego_dir.x, ego_dir.y]
        for i in range(len(self.zombie_cars)):
            if self.zombie_cars[i] == self.vehicle:
                continue
            z_pos = self.zombie_cars[i].get_location()
            dis = np.hypot(z_pos.x - self.current_loc.x, z_pos.y - self.current_loc.y)
            if dis < 50:
                vec_car = [z_pos.x - ego_pos[0], z_pos.y - ego_pos[1]]
                theta = common.cal_angle(vec_car, ego_vec)
                if self.check_onroad(self.zombie_cars[i], lane):
                    if theta <= math.pi/2:
                        forwards_cars.append([dis, i])
                    else:
                        backwards_cars.append([dis, i])

        if forwards_cars:
            forwards_cars.sort()
            lead_car = self.zombie_cars[forwards_cars[0][1]]
            pos_x = lead_car.get_transform().location.x
            pos_y = lead_car.get_transform().location.y
            length = lead_car.bounding_box.extent.x
            vel = np.hypot(lead_car.get_velocity().x, lead_car.get_velocity().y)
            lead_info = [pos_x, pos_y, length, vel]
            # self.bbox_display(lead_car, "blue")
        else:
            lead_info = None

        if backwards_cars:
            backwards_cars.sort()
            fol_car = self.zombie_cars[backwards_cars[0][1]]
            pos_x = fol_car.get_transform().location.x
            pos_y = fol_car.get_transform().location.y
            length = fol_car.bounding_box.extent.x
            vel = np.hypot(fol_car.get_velocity().x, fol_car.get_velocity().y)
            acc = np.hypot(fol_car.get_acceleration().x, fol_car.get_acceleration().y)
            fol_info = [pos_x, pos_y, length, vel, acc]
            # self.bbox_display(fol_car, "green")
        else:
            fol_info = None

        return lead_info, fol_info

    def check_onroad(self, vehicle, lane):
        for i in range(4):
            p = vehicle.get_physics_control().wheels[i].position/100
            wp = self.map.get_waypoint(p)
            if wp.lane_id == lane.lane_id:
                return True
        return False

    def lane_display(self, wp_list):
        # Legal Lane
        for wp in wp_list:
            if wp.lane_change == carla.LaneChange.Right:
                legal_pos_list_left = self.generate_position_list(self.local_wp(wp), 'left')
                self.draw_lane_line(legal_pos_list_left, 'left')
            elif wp.lane_change == carla.LaneChange.Left:
                legal_pos_list_right = self.generate_position_list(self.local_wp(wp), 'right')
                self.draw_lane_line(legal_pos_list_right, 'right')
            elif wp.lane_change == carla.LaneChange.NONE:
                legal_pos_list_left = self.generate_position_list(self.local_wp(wp), 'left')
                legal_pos_list_right = self.generate_position_list(self.local_wp(wp), 'right')
                self.draw_lane_line(legal_pos_list_left, 'left')
                self.draw_lane_line(legal_pos_list_right, 'right')

    def bbox_display(self, vehicle, color):
        if color == "red":
            _color = carla.Color(60, 10, 10, 0)
        elif color == "green":
            _color = carla.Color(10, 60, 10, 0)
        elif color == "blue":
            _color = carla.Color(10, 10, 60, 0)
        # Bounding Box
        bounding_box = vehicle.bounding_box
        bounding_box.location = vehicle.get_transform().location
        self.world.debug.draw_box(bounding_box, vehicle.get_transform().rotation,
                                  color=_color, life_time=self.show_dt)

    def ref_display(self, ref_x, ref_y):
        for n in range(0, len(ref_x), max(int(len(ref_x)/20), 1)):
            p = carla.Location()
            p.x = ref_x[n]
            p.y = ref_y[n]
            self.world.debug.draw_point(p, size=0.08, color = carla.Color(0, 0, 255), life_time=self.show_dt)

    def wp_side_extract(self, wp_list, wp, side):

        if side == 'right':
            while True:
                if (wp.lane_change == carla.LaneChange.Right
                        or wp.lane_change == carla.LaneChange.Both):
                    wp = wp.get_right_lane()
                    wp_list.append(wp)
                else:
                    break

        if side == 'left':
            while True:
                if (wp.lane_change == carla.LaneChange.Left
                        or wp.lane_change == carla.LaneChange.Both):
                    wp = wp.get_left_lane()
                    wp_list.append(wp)
                else:
                    break

        return wp_list

    def find_lanepoint_right(self, wp):
        location_drift = carla.Location(x=-np.sin(wp.transform.rotation.yaw / 180 * np.pi) * wp.lane_width / 2,
                                        y=np.cos(wp.transform.rotation.yaw / 180 * np.pi) * wp.lane_width / 2,
                                        z=0.2)
        lp = carla.Location(wp.transform.location + location_drift)
        return lp

    def find_lanepoint_left(self, wp):
        location_drift = carla.Location(x=np.sin(wp.transform.rotation.yaw / 180 * np.pi) * wp.lane_width / 2,
                                        y=-np.cos(wp.transform.rotation.yaw / 180 * np.pi) * wp.lane_width / 2,
                                        z=0.2)
        lp = carla.Location(wp.transform.location + location_drift)
        return lp

    def generate_position_list(self, wp_l, side='right'):
        if wp_l is None:
            return None
        else:
            pos_list = []
            if side == 'right':
                for i in range(len(wp_l)):
                    pos_list.append(self.find_lanepoint_right(wp_l[i]))
            elif side == 'left':
                for i in range(len(wp_l)):
                    pos_list.append(self.find_lanepoint_left(wp_l[i]))
            else:
                return None
        return pos_list

    def draw_lane_points(self, wp_l, side='right'):
        if wp_l and side == 'middle':
            for i in range(len(wp_l)):
                self.world.debug.draw_point(wp_l[i].transform.location +
                                            carla.Location(0, 0, 0.2), life_time=0.2)

        if wp_l and side == 'right':
            for i in range(len(wp_l)):
                self.world.debug.draw_point(self.find_lanepoint_right(wp_l[i]), life_time=0.2)

        elif wp_l and side == 'left':
            for i in range(len(wp_l)):
                self.world.debug.draw_point(self.find_lanepoint_left(wp_l[i]), life_time=0.2)

        return

    def draw_lane_line(self, pos_list, side='right'):
        if pos_list and side == 'right':
            for _loop in range(len(pos_list) // 2 - 1):
                self.world.debug.draw_line(pos_list[2 * _loop],
                                      pos_list[2 * _loop + 1],
                                      thickness=0.2,
                                      color=carla.Color(50, 0, 100, 0),
                                      life_time=0.2)
        if pos_list and side == 'left':
            for _loop in range(len(pos_list) // 2 - 1):
                self.world.debug.draw_line(pos_list[2 * _loop],
                                      pos_list[2 * _loop + 1],
                                      thickness=0.2,
                                      color=carla.Color(50, 0, 100, 0),
                                      life_time=0.2)
