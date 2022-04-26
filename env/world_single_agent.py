import carla
import random
# import scenarios
import numpy as np
from queue import Queue


class WorldInit:

    def __init__(self, args):
        self.client = carla.Client(args.host, args.port)
        self.client.set_timeout(3.0)
        self.world = self.client.load_world('Town04')

        self.original_settings = self.world.get_settings()
        self.dt = 0.1
        random.seed(3)

        settings = self.world.get_settings()
        settings.fixed_delta_seconds = self.dt
        settings.synchronous_mode = True
        self.world.apply_settings(settings)

        traffic_manager = self.client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)

        self.sensor_queue = Queue()

        self.spawn_points = self.world.get_map().get_spawn_points()
        spawn_point = self.spawn_points[1]
        self.spawn_points.remove(self.spawn_points[1])

        for p in self.spawn_points:
            dist = np.hypot(p.location.x - spawn_point.location.x,
                            p.location.y - spawn_point.location.y)
            if dist > 300:
                self.spawn_points.remove(p)

        waypoint = self.world.get_map().get_waypoint(spawn_point.location)
        spawn_point_leadcar = waypoint.next(30)[0].transform
        spawn_point_leadcar.location.z = spawn_point.location.z

        self.ego_car = self.add_ego_car(spawn_point, "50, 50, 200")
        self.zombie_cars = []
        self.add_zombie_cars(200)

        camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        self.camera = self.world.spawn_actor(camera_bp,
                                   carla.Transform(carla.Location(x=-5.5, z=2.5),
                                                   carla.Rotation(pitch=8.0)),
                                   self.ego_car,
                                   carla.AttachmentType.SpringArm)
        self.camera.listen(lambda image: self.sensor_queue.put(image.frame))

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

    def __del__(self):
        self.world.apply_settings(self.original_settings)
        print('done')