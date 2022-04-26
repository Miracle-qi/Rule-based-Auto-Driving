# This is a sample Python script.

import argparse

try:
    import numpy as np
    import sys
    from os import path as osp
except ImportError:
    raise RuntimeError('import error!')

import matplotlib.pyplot as plt
from queue import Empty
from env.world_single_agent import WorldInit
from agent.feature import FeatureExt
from agent.path_planner import *
import agent.velocity_planner as velocity_planner
from agent.linear_MPC import MPC
import agent.vehicle_model as model
from agent.rule_decision import *
from agent.pid import *


def main():

    argparser = argparse.ArgumentParser(description='Carla ArgParser practice')
    argparser.add_argument('--host', metavar='H', default='127.0.0.1', help='IP of the host server')
    argparser.add_argument('-p', '--port', default=2000, type=int, help='TCP port to listen to')
    argparser.add_argument('-a', '--autopilot', action='store_true', help='enable autopilot')
    args = argparser.parse_args()
    running = True

    try:
        env = WorldInit(args)
        fea_extract = FeatureExt(env, env.ego_car)
        car_model = model.Vehicle(env.ego_car, env.dt, 12)
        dec_maker = RuleBased(car_model, fea_extract, env.dt)
        path = path_planner(env, car_model, fea_extract)
        mpc = MPC(car_model)
        pid = PID_controller(car_model, env.dt)


        plt.ion()
        plt.figure(1)
        v_his, ref_v = [], []

        tic = 0
        while running:
            tic += 1

            env.world.tick()
            if tic > 0:
                car_model.update()
                spectator = env.world.get_spectator()
                transform = env.ego_car.get_transform()
                current_loc = transform.location
                spectator.set_transform(carla.Transform(current_loc +
                                                        carla.Location(x=-20, z=50),
                                                        carla.Rotation(pitch=-60)))

                fea_extract.update()
                fea_extract.lane_display(fea_extract.wp_list)
                # if (car_model.status == STATUS.FOLLOWING):
                ref_vel = dec_maker.decision()
                print("Reference velocity:", ref_vel)
                rx, ry, ryaw, s_sum = path.update()
                fea_extract.ref_display(rx, ry)
                # poly_coe = velocity_planner.speed_profile_quinticPoly(car_model, 0, 0, dec_maker.stop_dist)
                poly_coe = velocity_planner.speed_profile_uniform(ref_vel)
                control_comd = mpc.update(rx, ry, ryaw, poly_coe)
                # control_comd = pid.update(rx, ry, ryaw, poly_coe)
                env.ego_car.apply_control(control_comd)

                print("yaw:", ryaw[0], car_model.yaw)

                v_his.append(car_model.v)
                ref_v.append(ref_vel)
                plt.plot(v_his, linestyle='--', color='b')
                plt.plot(ref_v, linestyle='--', color='r')
                plt.pause(0.0001)

                print("----------------------")

            try:
                env.sensor_queue.get()
            except Empty:
                print('some of the sensor information is missed')
    finally:
        env.__del__()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Exit by user')


