# -*- coding: utf-8 -*-

"""Revised automatic control
"""
# Author: Runsheng Xu <rxx3386@ucla.edu>
# License: MIT

import os
import random
import sys

import carla

from agents.navigation.behavior_agent import BehaviorAgent


def main():
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(2.0)

        # Retrieve the world that is currently running
        world = client.get_world()

        origin_settings = world.get_settings()

        # set sync mode
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        blueprint_library = world.get_blueprint_library()

        # read all valid spawn points
        all_default_spawn = world.get_map().get_spawn_points()
        # randomly choose one as the start point
        spawn_point = random.choice(all_default_spawn) if all_default_spawn else carla.Transform()

        # create the blueprint library
        ego_vehicle_bp = blueprint_library.find('vehicle.lincoln.mkz2017')
        ego_vehicle_bp.set_attribute('color', '0, 0, 0')
        # spawn the vehicle
        vehicle = world.spawn_actor(ego_vehicle_bp, spawn_point)

        # we need to tick the world once to let the client update the spawn position
        world.tick()

        # create the behavior agent
        # BehaviorAgent 在构建时需要两个输入，一个是属于Actor class的vehicle,
        # 另外一个就是车辆驾驶风格（string type).
        agent = BehaviorAgent(vehicle, behavior='normal')

        # set the destination spot
        spawn_points = world.get_map().get_spawn_points()
        random.shuffle(spawn_points)

        # to avoid the destination and start position same
        if spawn_points[0].location != agent.vehicle.get_location():
            destination = spawn_points[0]
        else:
            destination = spawn_points[1]

        # generate the route
        agent.set_destination(agent.vehicle.get_location(), destination.location, clean=True)

        while True:
            # agent.update_information(vehicle): BehaviorAgent更新汽车的实时信息，方便更新行为规划
            agent.update_information(vehicle)

            world.tick()  # world.tick()->仿真世界运行一个步长
            
            if len(agent._local_planner.waypoints_queue)<1:
                print('======== Success, Arrivied at Target Point!')
                break
                
            # top view
            spectator = world.get_spectator()
            transform = vehicle.get_transform()
            spectator.set_transform(carla.Transform(transform.location + carla.Location(z=40),
                                                    carla.Rotation(pitch=-90)))

            speed_limit = vehicle.get_speed_limit()
            agent.get_local_planner().set_speed(speed_limit)

            # 最核心的一步，更新BehaviorAgent的信息之后，agent运行新的一步规划，并产生相应的控制命令
            """
            我们初始化了BehaviorAgent, 制定好了全局路线，并将当前server的重要信息更新给了agent, 
            现在所有的信息都一应俱全，我们的agent终于开始行为规划了！CARLA中的行为规划可大致分为五步：
            1）针对交通信号灯的行为规划，2）针对行人的行为规划，3）针对路上其它车辆的行为规划，
            4）交叉口的行为规划， 5）正常驾驶的行为规划。如下方代码所示，最外层的代码只有两行。
            """
            control = agent.run_step(debug=True)
            # 汽车执行产生的控制命令，在仿真世界运行
            vehicle.apply_control(control)

    finally:
        world.apply_settings(origin_settings)
        vehicle.destroy()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')
