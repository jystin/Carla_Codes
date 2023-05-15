import carla
import random
import os
from queue import Queue
from queue import Empty
from carla import Transform, Location, Rotation
import time



# callback function to save the info of sensors
def sensor_callback(sensor_data, sensor_queue, sensor_name):
    if 'lidar1' in sensor_name:
        sensor_data.save_to_disk(os.path.join('outputs_cline/lidar1', '%06d.ply' % sensor_data.frame))
    if 'lidar2' in sensor_name:
        sensor_data.save_to_disk(os.path.join('outputs_cline/lidar2', '%06d.ply' % sensor_data.frame))
    if 'camera' in sensor_name:
        sensor_data.save_to_disk(os.path.join('outputs_cline/camera', '%06d.png' % sensor_data.frame))
    sensor_queue.put((sensor_data.frame, sensor_name))


def main():
    actor_list = []
    sensor_list = []
    box_x = 5
    try:
        # First of all, we need to create the client that will send the requests, assume port is 2000
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)

        # Retrieve the world that is currently running
        world = client.get_world()
        # world = client.load_world('Town02') # you can also retrive another world by specifically defining
        blueprint_library = world.get_blueprint_library()
        # all_actors = world.get_actors()
        # print(all_actors)
        # for actor in all_actors:
        #     print(actor.id)
        # blueprints = [bp for bp in world.get_blueprint_library().filter('*')]
        # for blueprint in blueprints:
        #     if blueprint.id.startswith("static"):
        #         print(blueprint.id)
        #         for attr in blueprint:
        #             print('  - {}'.format(attr))
        # static.prop.box02 / static.prop.mesh
        # Set weather for your world
        # weather = carla.WeatherParameters(cloudiness=10.0,
        #                                   precipitation=10.0,
        #                                   fog_density=10.0)
        # world.set_weather(weather)

        # set synchorinized mode
        original_settings = world.get_settings()
        settings = world.get_settings()
        settings.fixed_delta_seconds = 0.05
        settings.synchronous_mode = True
        world.apply_settings(settings)

        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)

        # create sensor queue
        sensor_queue = Queue()

        # create detected object
        detected_object_bp = blueprint_library.find('static.prop.box02')
        detected_object_transform = carla.Transform(carla.Location(x=box_x, y=204.5, z=1.1), carla.Rotation(0, 0, 0))
        detected_object = world.spawn_actor(detected_object_bp, detected_object_transform)
        # actor_list.append(detected_object)

        # create the ego vehicle
        ego_vehicle_bp = blueprint_library.find('vehicle.mercedes.coupe_2020')
        # black color
        ego_vehicle_bp.set_attribute('color', '0, 0, 0')
        # get a random valid occupation in the world                                                    # y   z   x
        transform = carla.Transform(carla.Location(x=5.989376, y=207.505676, z=0.275307), carla.Rotation(pitch=0.000000, yaw=0.000000, roll=0.000000))
        # transform = random.choice(world.get_map().get_spawn_points()) 598.937622  13.181384
        # spawn the vehilce
        ego_vehicle = world.spawn_actor(ego_vehicle_bp, transform)
        # set the vehicle autopilot mode
        # ego_vehicle.set_autopilot(True)
        # collect all actors to destroy when we quit the script
        actor_list.append(ego_vehicle)

        # create directory for outputs_cline
        output_path = 'outputs_cline/lidar1'
        if not os.path.exists(output_path):
            os.makedirs(output_path)
        output_path = 'outputs_cline/lidar2'
        if not os.path.exists(output_path):
            os.makedirs(output_path)

        # add a camera
        # camera_bp = blueprint_library.find('sensor.camera.rgb')
        # # camera relative position related to the vehicle
        # camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        # camera = world.spawn_actor(camera_bp, camera_transform, attach_to=ego_vehicle)
        # # set the callback function
        # camera.listen(lambda image: sensor_callback(image, sensor_queue, "camera"))
        # sensor_list.append(camera)

        # we also add a lidar on it
        lidar_bp1 = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp2 = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp1.set_attribute('channels', str(64))
        lidar_bp1.set_attribute('points_per_second', str(180000))
        lidar_bp1.set_attribute('rotation_frequency', str(40))
        lidar_bp1.set_attribute('range', str(20))
        lidar_bp2.set_attribute('channels', str(64))
        lidar_bp2.set_attribute('points_per_second', str(180000))
        lidar_bp2.set_attribute('rotation_frequency', str(40))
        lidar_bp2.set_attribute('range', str(20))

        # set the relative location
        lidar_location = carla.Location(0, 0, 3)
        lidar_rotation = carla.Rotation(0, 0, 0)
        lidar_transform1 = carla.Transform(lidar_location, lidar_rotation)
        lidar_transform2 = carla.Transform(carla.Location(7.2, 0, 3), carla.Rotation(0, 0, 0))
        # spawn the lidar
        lidar1 = world.spawn_actor(lidar_bp1, lidar_transform1, attach_to=ego_vehicle)
        lidar2 = world.spawn_actor(lidar_bp1, lidar_transform2, attach_to=ego_vehicle)
        lidar1.listen(
            lambda point_cloud: sensor_callback(point_cloud, sensor_queue, "lidar1"))
        lidar2.listen(
            lambda point_cloud: sensor_callback(point_cloud, sensor_queue, "lidar2"))
        sensor_list.append(lidar1)
        sensor_list.append(lidar2)

        while True:
            world.tick()
            # set the sectator to follow the ego vehicle
            detected_object.destroy()
            box_x += 0.1
            detected_object_bp = blueprint_library.find('static.prop.box02')
            detected_object_transform = carla.Transform(carla.Location(x=box_x, y=204.5, z=1.1),
                                                        carla.Rotation(0, 0, 0))
            detected_object = world.spawn_actor(detected_object_bp, detected_object_transform)
            # print(detected_object.get_location())
            # print(ego_vehicle.get_location())
            # actor_list.append(detected_object)

            spectator = world.get_spectator()
            transform = ego_vehicle.get_transform()
            spectator.set_transform(carla.Transform(transform.location + carla.Location(z=20),
                                                    carla.Rotation(pitch=-90)))

            # As the queue is blocking, we will wait in the queue.get() methods
            # until all the information is processed and we continue with the next frame.
            try:
                for i in range(0, len(sensor_list)):
                    s_frame = sensor_queue.get(True, 1.0)
                    # print("    Frame: %d   Sensor: %s" % (s_frame[0], s_frame[1]))

            except Empty:
                print("   Some of the sensor information is missed")

    finally:
        world.apply_settings(original_settings)
        print('destroying actors')
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        for sensor in sensor_list:
            sensor.destroy()
        detected_object.destroy()
        print('done.')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')