# This is a sample Python script.

import argparse

try:
    import numpy as np
    import sys
    from os import path as osp
except ImportError:
    raise RuntimeError('import error!')

from queue import Empty
from env.world_multi_agent import WorldInit
from agent.feature import FeatureExt
from agent.path_planner import *
import agent.velocity_planner as velocity_planner
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

        fea_extract, fea_extract, car_model, dec_maker, path, pid = [], [], [], [], [], []
        for n in range(len(env.ego_cars)):
            fea_extract.append(FeatureExt(env, env.ego_cars[n]))
            car_model.append(model.Vehicle(env.ego_cars[n], env.dt, env.velocity_list[n]))
            dec_maker.append(RuleBased(car_model[n], fea_extract[n], env.dt))
            path.append(path_planner(env, car_model[n], fea_extract[n]))
            pid.append(PID_controller(car_model[n], env.dt))

        while running:
            env.world.tick()
            spectator = env.world.get_spectator()
            transform = env.ego_cars[0].get_transform()
            current_loc = transform.location
            spectator.set_transform(carla.Transform(current_loc +
                                                    carla.Location(x=-20, z=70),
                                                    carla.Rotation(pitch=-70)))

            for n in range(len(env.ego_cars)):
                print("Vehicle index:", n)
                car_model[n].update()
                fea_extract[n].update()
                # fea_extract[n].lane_display(fea_extract[n].wp_list)
                ref_vel = dec_maker[n].decision()
                print("ref:", ref_vel)
                rx, ry, ryaw, s_sum = path[n].update()
                fea_extract[n].ref_display(rx, ry)
                poly_coe = velocity_planner.speed_profile_uniform(ref_vel)
                control_comd = pid[n].update(rx, ry, ryaw, poly_coe)
                env.ego_cars[n].apply_control(control_comd)
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


