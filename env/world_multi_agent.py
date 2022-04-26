import carla
import random
# import scenarios
import numpy as np
from queue import Queue

class WorldInit:

    def __init__(self, args):
        self.client = carla.Client(args.host, args.port)
        self.client.set_timeout(3.0)
        self.world = self.client.load_world('Town05')
        self.world.freeze_all_traffic_lights(True)

        self.original_settings = self.world.get_settings()
        self.dt = 0.12
        random.seed(3) # feasible seeds: 0, 2, 3, 4

        settings = self.world.get_settings()
        settings.fixed_delta_seconds = self.dt
        settings.synchronous_mode = True
        self.world.apply_settings(settings)

        traffic_manager = self.client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)

        self.sensor_queue = Queue()

        self.spawn_points = self.world.get_map().get_spawn_points()
        spawn_point = self.spawn_points[8]
        self.spawn_points.remove(self.spawn_points[1])
        self.velocity_list = []

        # for p in self.spawn_points:
        #     dist = np.hypot(p.location.x - spawn_point.location.x,
        #                     p.location.y - spawn_point.location.y)
        #     if dist > 30:
        #         self.spawn_points.remove(p)

        waypoint = self.world.get_map().get_waypoint(spawn_point.location)
        spawn_point_leadcar = waypoint.next(30)[0].transform
        spawn_point_leadcar.location.z = spawn_point.location.z

        self.ego_cars = []
        self.add_ego_cars(spawn_point, 4)
        self.zombie_cars = self.ego_cars

        camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        self.camera = self.world.spawn_actor(camera_bp,
                                   carla.Transform(carla.Location(x=-5.5, z=2.5),
                                                   carla.Rotation(pitch=8.0)),
                                   self.ego_cars[0],
                                   carla.AttachmentType.SpringArm)
        self.camera.listen(lambda image: self.sensor_queue.put(image.frame))

    def add_ego_cars(self, s_point, count):
        spawn_points = []

        cur_lane = self.world.get_map().get_waypoint(s_point.location)
        right_lane = cur_lane.get_right_lane()
        left_lane = cur_lane.get_left_lane()
        lleft_lane = left_lane.get_left_lane()
        rright_lane = right_lane.get_right_lane()

        for n in range(3):
            spawn_points.append([cur_lane.next(n * 10 + 0.1)[0], 12 - n * 1])
            spawn_points.append([left_lane.next(n * 15+ 20.1)[0], 15 - n * 3])
            # spawn_points.append([lleft_lane.next(n * 15+ 0.1)[0], 15 - n * 3])
            # spawn_points.append([right_lane.next(n * 15 + 0.1)[0], 15 - n * 1])
            # spawn_points.append([rright_lane.next(n * 15 + 0.1)[0], 15 - n * 0.5])

        # blueprints = self.world.get_blueprint_library().filter('vehicle.*')
        # blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
        # blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
        model3_bp = self.world.get_blueprint_library().find('vehicle.tesla.model3')
        model3_bp.set_attribute('color', "200,100,100")
        random.shuffle(spawn_points)
        for [point, v] in spawn_points:
            if count <= 0:
                break
            else:
                # blueprint = random.choice(blueprints)
                feasible_p = carla.Transform(point.transform.location + carla.Location(z=s_point.location.z),
                                             point.transform.rotation)
                vehicle = self.world.try_spawn_actor(model3_bp, feasible_p)
                if vehicle is not None:
                    self.velocity_list.append(v)
                    vehicle.set_simulate_physics(True)
                    vehicle.set_autopilot(False)
                    self.ego_cars.append(vehicle)
                    count -= 1

    def add_ego_car(self, spawn_point, color):
        model3_bp = self.world.get_blueprint_library().find('vehicle.tesla.model3')
        model3_bp.set_attribute('color', color)
        # model3_spawn_point = np.random.choice(spawn_points)
        ego_car = self.world.spawn_actor(model3_bp, spawn_point)
        ego_car.set_autopilot(False)
        return ego_car

    def add_zombie_cars(self, count=5):

        random.shuffle(self.spawn_points)
        blueprints = self.world.get_blueprint_library().filter('vehicle.*')
        blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
        blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
        for spawn_point in self.spawn_points:
            if count <= 0:
                break
            else:
                blueprint = random.choice(blueprints)
                vehicle = self.world.try_spawn_actor(blueprint, spawn_point)
                if vehicle is not None:
                    vehicle.set_simulate_physics(True)
                    vehicle.set_autopilot(True)
                    self.zombie_cars.append(vehicle)
                    count -= 1


    # def scenario_init(self, name, map, world):
    #     if name == 'OtherLeadingVehicle':
    #         scenario = OtherLeadingVehicle
    #     elif name == 'CarFollowing':
    #         scenario = CarFollowing
    #     elif name == 'OtherLeadingVehicle_FullMap':
    #         scenario = OtherLeadingVehicle_FullMap
    #     elif name == 'Cross_Join':
    #         scenario = Cross_Join
    #     elif name == 'Ring_Join':
    #         scenario = Ring_Join
    #     elif name == 'Straight_Follow_Single':
    #         scenario = Straight_Follow_Single
    #     elif name == 'Straight_Follow_Double':
    #         scenario = Straight_Follow_Double
    #     elif name == 'Cross_Follow':
    #         scenario = Cross_Follow
    #     elif name == 'Cross_Turn_Left':
    #         scenario = Cross_Turn_Left
    #     elif name == 'Cross_Turn_Right':
    #         scenario = Cross_Turn_Right
    #     elif name == 'OverTake':
    #         scenario = OverTake
    #     else:
    #         raise NotImplementedError('Scenario does not exist!')
    #     if name == 'OverTake':
    #         self.scenario_now = scenario(name, map, world, self.checkkeys)
    #     else:
    #         self.scenario_now = scenario(name, map, world)
    #     self.vehicle = self.scenario_now.hero_car




    def __del__(self):
        self.world.apply_settings(self.original_settings)
        print('done')