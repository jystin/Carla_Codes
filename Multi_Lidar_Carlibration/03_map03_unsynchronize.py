import carla
import random
import os
import time
from queue import Queue


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

    try:
        # First of all, we need to create the client that will send the requests, assume port is 2000
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)

        # Retrieve the world that is currently running
        world = client.get_world()
        # world = client.load_world('Town02') # you can also retrive another world by specifically defining
        blueprint_library = world.get_blueprint_library()

        sensor_queue = Queue()
        ego_vehicle_bp = blueprint_library.find('vehicle.mercedes.coupe_2020')
        # black color
        ego_vehicle_bp.set_attribute('color', '0, 0, 0')
        # get a random valid occupation in the world                                                    # y   z   x
        transform = carla.Transform(carla.Location(x=5.989376, y=207.505676, z=0.275307),
                                    carla.Rotation(pitch=0.000000, yaw=-0.142975, roll=0.000000))
        # transform = random.choice(world.get_map().get_spawn_points()) 598.937622  13.181384
        # spawn the vehilce
        ego_vehicle = world.spawn_actor(ego_vehicle_bp, transform)
        # set the vehicle autopilot mode
        ego_vehicle.set_autopilot(True)
        # collect all actors to destroy when we quit the script
        actor_list.append(ego_vehicle)
        # vehicle = blueprint_library.filter('vehicle.*.*')
        # is_bike = [vehicle.get_attribute('number_of_wheels') == 2]
        # if (is_bike):
        #     vehicle.set_attribute('color', '255,0,0')
        output_path = 'outputs_cline/lidar1'
        if not os.path.exists(output_path):
            os.makedirs(output_path)
        output_path = 'outputs_cline/lidar2'
        if not os.path.exists(output_path):
            os.makedirs(output_path)

        lidar_bp1 = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp2 = blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp1.set_attribute('channels', str(64))
        lidar_bp1.set_attribute('points_per_second', str(180000))
        lidar_bp1.set_attribute('rotation_frequency', str(60))
        lidar_bp1.set_attribute('range', str(20))
        lidar_bp2.set_attribute('channels', str(64))
        lidar_bp2.set_attribute('points_per_second', str(180000))
        lidar_bp2.set_attribute('rotation_frequency', str(60))
        lidar_bp2.set_attribute('range', str(20))

        lidar_location = carla.Location(0, 0, 3)
        lidar_rotation = carla.Rotation(0, 0, 0)
        lidar_transform1 = carla.Transform(lidar_location, lidar_rotation)
        lidar_transform2 = carla.Transform(carla.Location(3, 0, 3), carla.Rotation(0, 0, 0))
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
            # set the sectator to follow the ego vehicle
            spectator = world.get_spectator()
            transform = ego_vehicle.get_transform()
            spectator.set_transform(carla.Transform(transform.location + carla.Location(z=20),
                                                    carla.Rotation(pitch=-90)))

    finally:
        print('destroying actors')
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        for sensor in sensor_list:
            sensor.destroy()
        print('done.')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')