import carla
import random
import os
import time

# 连接服务器，必须在提前UE界面中点击运行按钮
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

# 获取当前世界
world = client.get_world()

# 客户端还可以获取可用地图列表来更改当前地图。这将摧毁当前的世界并创造一个新的世界。
# print(client.get_available_maps())
# world = client.load_world('Town01')
# client.reload_world() creates a new instance of the world with the same map.

# 设置天气
# weather = carla.WeatherParameters(
#     cloudiness=80.0,
#     precipitation=30.0,
#     sun_altitude_angle=70.0)
# world.set_weather(weather)
# time.sleep(1)
# # 可以设置一些已经存在的天气
# world.set_weather(carla.WeatherParameters.WetCloudySunset)
# # 获取当前天气
# print(world.get_weather())

# Actor and Blueprint
blueprint_library = world.get_blueprint_library()
# Find a specific blueprint.
collision_sensor_bp = blueprint_library.find('sensor.other.collision')
# Choose a vehicle blueprint at random.
vehicle_bp = random.choice(blueprint_library.filter('vehicle.*.*'))

# 创建actor只需要一个蓝图和一个transform
# 如果在指定位置发生碰撞，actor 将不会生成。
# transform = carla.Transform(carla.Location(x=230, y=195, z=40), carla.Rotation(yaw=180))
# map.get_spawn_points() 对于车辆。返回推荐的生成点列表
transform = world.get_map().get_spawn_points()
ego_vehicle = world.spawn_actor(vehicle_bp, transform)

# vehicle = blueprint_library.filter('vehicle.*.*')
# is_bike = [vehicle.get_attribute('number_of_wheels') == 2]
# if (is_bike):
#     vehicle.set_attribute('color', '255,0,0')


# 拿到这个世界所有物体的蓝图
# blueprint_library = world.get_blueprint_library()
# # 从浩瀚如海的蓝图中找到奔驰的蓝图
# ego_vehicle_bp = blueprint_library.find('vehicle.mercedes.coupe_2020')
# # 给我们的车加上特定的颜色
# ego_vehicle_bp.set_attribute('color', '0, 0, 0')
# # 找到所有可以作为初始点的位置并随机选择一个
# transform = random.choice(world.get_map().get_spawn_points())
# print(transform.location)
# # 在这个位置生成汽车
# ego_vehicle = world.spawn_actor(ego_vehicle_bp, transform)
# # 再给它挪挪窝
# location = ego_vehicle.get_location()
# location.x += 10.0
# ego_vehicle.set_location(location)
# 把它设置成自动驾驶模式
# ego_vehicle.set_autopilot(True)
# 我们可以甚至在中途将这辆车“冻住”，通过抹杀它的物理仿真
# actor.set_simulate_physics(False)

# camera_bp = blueprint_library.find('sensor.camera.rgb')
# camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
# camera = world.spawn_actor(camera_bp, camera_transform, attach_to=ego_vehicle)
# output_path = '/home/justyn/桌面/Carla学习/传感器数据/camera'
# camera.listen(lambda image: image.save_to_disk(os.path.join(output_path, '%06d.png' % image.frame)))


# 当这个脚本运行完后要记得将这个汽车销毁掉，否则它会一直存在于仿真世界，可能影响其他脚本的运行哦。
# 注销单个Actor
ego_vehicle.destroy()
# # 如果你有多个Actor 存在cwlist里，想一起销毁。
# # client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])

