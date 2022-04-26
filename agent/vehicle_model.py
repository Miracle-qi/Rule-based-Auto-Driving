import math
import numpy as np
import common


class STATUS:
    FOLLOWING = 0
    START_LANE_CHANGE_L = 1
    LANE_CHANGING_L = 2
    START_LANE_CHANGE_R = 3
    LANE_CHANGING_R = 4
    STOPPING = 5


class Vehicle:

    def __init__(self, vehicle, dt, target_vel):
        self.vehicle = vehicle
        self.target_vel = target_vel
        self.dt = dt
        self.steer_max = math.pi / 5
        self.speed_max = 40
        self.acc_max = 8
        self.steer_max = np.deg2rad(45.0)
        self.steer_change_max = np.deg2rad(15.0)  # maximum steering speed [rad/s]
        wb_vec = vehicle.get_physics_control().wheels[0].position - vehicle.get_physics_control().wheels[2].position
        self.wheelbase = np.sqrt(wb_vec.x**2 + wb_vec.y**2 + wb_vec.z**2)/100
        self.merge_length = 0
        self.shape = [self.vehicle.bounding_box.extent.x, self.vehicle.bounding_box.extent.y,
                      self.vehicle.bounding_box.extent.z]

        self.x = None
        self.y = None
        self.v = None
        self.acc = None
        self.yaw = None
        self.steer = None
        self.status = STATUS.FOLLOWING
        self.update()  # initialize

    def update(self):
        self.x = self.vehicle.get_location().x
        self.y = self.vehicle.get_location().y

        _v = self.vehicle.get_velocity()
        _acc = self.vehicle.get_acceleration()
        self.v = np.sqrt(_v.x ** 2 + _v.y ** 2)
        self.acc = np.sqrt(_acc.x ** 2 + _acc.y ** 2)
        self.steer = self.vehicle.get_control().steer
        self.merge_length = max(4 * self.v, 12)

        wb_vec = self.vehicle.get_physics_control().wheels[0].position - \
                 self.vehicle.get_physics_control().wheels[2].position
        self.yaw = math.atan2(wb_vec.y, wb_vec.x)

    def predict(self, a, delta):
        '''Bicycle Model for vehicles'''
        # delta = self.limit_input_delta(delta)
        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt
        self.yaw += self.v / self.wheelbase * math.tan(delta) * self.dt
        self.v += a * self.dt
        # self.v = self.limit_speed(self.v)

    def limit_input_delta(self, delta):
        if delta >= self.steer_max:
            return self.steer_max

        if delta <= -self.steer_max:
            return -self.steer_max

        return delta

    def limit_speed(self, v):
        if v >= self.speed_max:
            return self.speed_max

        if v <= -self.speed_max:
            return -self.speed_max

        return v