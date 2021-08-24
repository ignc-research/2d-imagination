#!/usr/bin/env python
import numpy as np
import rospy
import os
import rospkg
import sys
import time
import copy
import math
from std_srvs.srv import Trigger
from pedsim_srvs.srv import SpawnPeds
from pedsim_srvs.srv import SpawnInteractiveObstacles
from pedsim_srvs.srv import SpawnObstacle
from pedsim_msgs.msg import Ped
from pedsim_msgs.msg import InteractiveObstacle
from pedsim_msgs.msg import LineObstacles
from pedsim_msgs.msg import LineObstacle
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from flatland_msgs.srv import SpawnModels
from flatland_msgs.msg import Model

def get_yaml_path_from_type(ped_type):
    rospack = rospkg.RosPack()
    if ped_type == "adult":
        return os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles", "person_two_legged.model.yaml")
    elif ped_type == "child":
        return os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles", "person_two_legged.model.yaml")
    elif ped_type == "elder":
        return os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles", "person_two_legged.model.yaml")
    elif ped_type == "forklift":
        return os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles", "big_forklift.model.yaml")
    elif ped_type == "servicerobot":
        return os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles", "servicerobot.model.yaml")
    elif ped_type == "shelf":
        return os.path.join(rospack.get_path("simulator_setup"), "obstacles", "shelf.yaml")
    #elif ped_type == "table":
    #    return os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "table1_polygon_4legs_circle.yaml")
    raise Exception("unknown ped type:", ped_type)

def get_default_ped(id, ped_type, yaml_path, pos, waypoints):
    ped = Ped()
    ped.id = id
    ped.pos = pos
    ped.type = ped_type
    ped.yaml_file = yaml_path
    ped.number_of_peds = 1
    ped.vmax = 1.0

    ped.chatting_probability = 0.01
    ped.tell_story_probability = 0.01
    ped.group_talking_probability = 0.01
    ped.talking_and_walking_probability = 0.01
    ped.requesting_service_probability = 0.01
    ped.requesting_guide_probability = 0.01
    ped.requesting_follower_probability = 0.01

    ped.talking_base_time = 10.0
    ped.tell_story_base_time = 10.0
    ped.group_talking_base_time = 10.0
    ped.talking_and_walking_base_time = 6.0
    ped.receiving_service_base_time = 20.0
    ped.requesting_service_base_time = 30.0

    ped.max_talking_distance = 2.0
    ped.max_servicing_radius = 10.0

    ped.force_factor_desired = 1.0
    ped.force_factor_obstacle = 1.0
    ped.force_factor_social = 5.0

    ped.waypoints = waypoints
    ped.waypoint_mode = 0
    return ped

def get_random_shelf():
    msg = InteractiveObstacle()   
    msg.pose = Pose()
    msg.pose.position.x = np.random.uniform(low=-2.0, high=15.0)
    msg.pose.position.y = np.random.uniform(low=-2.0, high=15.0)
    msg.pose.position.z = np.random.uniform(low=-2.0, high=15.0)
    msg.interaction_radius = 4.0
    msg.type = "shelf"
    msg.yaml_path = get_yaml_path_from_type("shelf")
    return msg

def social_force_test():
    time.sleep(2)
    respawn_peds_service_name = "pedsim_simulator/respawn_peds"
    rospy.wait_for_service(respawn_peds_service_name, 6.0)
    respawn_ped_srv = rospy.ServiceProxy(respawn_peds_service_name, SpawnPeds)

    peds = []

    ped = get_default_ped(
            id = 1,
            ped_type = "adult",
            yaml_path = get_yaml_path_from_type("adult"),
            pos = Point(10, 3, 0.1),
            waypoints = [Point(10, 10, 0.1), Point(10, 3, 0.1)]
        )
    ped.chatting_probability = 0
    ped.talking_and_walking_probability = 0
    ped.requesting_service_probability = 0
    ped.requesting_guide_probability = 0
    ped.force_factor_social = 20.0
    peds.append(ped)

    ped = get_default_ped(
            id = 2,
            ped_type = "adult",
            yaml_path = get_yaml_path_from_type("adult"),
            pos = Point(10, 10, 0.1),
            waypoints = [Point(10, 3, 0.1), Point(10, 10, 0.1)]
        )
    ped.chatting_probability = 0
    ped.talking_and_walking_probability = 0
    ped.requesting_service_probability = 0
    ped.force_factor_social = 20.0
    peds.append(ped)

    response = respawn_ped_srv.call(peds)
    print("successfully spawned peds" if response.success else "failed")

def crowd_test():
    time.sleep(2)
    respawn_peds_service_name = "pedsim_simulator/respawn_peds"
    rospy.wait_for_service(respawn_peds_service_name, 6.0)
    respawn_ped_srv = rospy.ServiceProxy(respawn_peds_service_name, SpawnPeds)

    peds = []

    ped = get_default_ped(
            id = 1,
            ped_type = "adult",
            yaml_path = get_yaml_path_from_type("adult"),
            pos = Point(5, 5, 0.1),
            waypoints = [Point(10, 10, 0.1), Point(10, 3, 0.1), Point(3, 10, 0.1), Point(3, 3, 0.1)]
        )
    ped.number_of_peds = 8
    ped.waypoint_mode = 1
    ped.chatting_probability = 0
    ped.talking_and_walking_probability = 0
    ped.tell_story_probability = 0
    ped.group_talking_probability = 0.01
    ped.group_talking_base_time = 300.0
    ped.requesting_service_probability = 0
    ped.requesting_guide_probability = 0
    ped.force_factor_social = 1.0
    peds.append(ped)

    response = respawn_ped_srv.call(peds)
    print("successfully spawned peds" if response.success else "failed")

def shelves_test():
    time.sleep(2)
    rospack = rospkg.RosPack()
    respawn_peds_service_name = "pedsim_simulator/respawn_peds"
    rospy.wait_for_service(respawn_peds_service_name, 6.0)
    respawn_ped_srv = rospy.ServiceProxy(respawn_peds_service_name, SpawnPeds)

    peds = []

    ped2 = Ped()
    ped2.id = 2
    ped2.pos = Point(-1, 3, 0)
    ped2.type = "forklift"
    ped2.number_of_peds = 1
    ped2.vmax = 2.0
    ped2.force_factor_desired = 1.0
    ped2.force_factor_obstacle = 1.0
    ped2.force_factor_social = 1.0
    ped2.waypoint_mode = 0
    ped2.waypoints = [Point(7, 3, 0.1), Point(9, 11, 0.1), Point(5, 5, 0.1)]
    ped2.yaml_file = os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles", "big_forklift.model.yaml")
    peds.append(ped2)

    response = respawn_ped_srv.call(peds)
    print("successfully spawned peds" if response.success else "failed")


    respawn_interactive_obstacles_service_name = "pedsim_simulator/respawn_interactive_obstacles"
    rospy.wait_for_service(respawn_interactive_obstacles_service_name, 6.0)
    respawn_interactive_obstacles_client = rospy.ServiceProxy(respawn_interactive_obstacles_service_name, SpawnInteractiveObstacles)

    obstacle = InteractiveObstacle()
    obstacle.name = "wp1"
    obstacle.type = "shelf"
    obstacle.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "obstacles", "shelf.yaml")
    obstacle.pose.orientation.w = 1
    obstacle.pose.position = Point(10.0, 14.0, 0)
    obstacle.interaction_radius = 5

    obstacle2 = copy.deepcopy(obstacle)
    obstacle2.name = "wp2"
    obstacle.pose.position = Point(10.0, 3.0, 0)

    #obstacle3 = InteractiveObstacle()
    #obstacle3.name = "wp3"
    #obstacle3.type = "table"
    #obstacle3.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "table1_polygon_4legs_circle.yaml")
    #obstacle3.pose.orientation.w = 1
    #obstacle3.pose.position = Point(20.0, 14.0, 0)
    #obstacle3.interaction_radius = 5

    #respawn_interactive_obstacles_client.call([obstacle, obstacle2, obstacle3])
    respawn_interactive_obstacles_client.call([obstacle, obstacle2])

def tables_test():
    time.sleep(2)
    spawn_model_service_name = "spawn_models"
    rospy.wait_for_service(spawn_model_service_name, 6.0)
    spawn_model_srv = rospy.ServiceProxy(spawn_model_service_name, SpawnModels)

    # the tables and the chairs were scaled, because they should be much bigger then the robot turtlebot3 with size LxWxH = 138mm x 178mm x 192mm
    # Important: the obstacles should be name like this:
    # -> for a table: '[global_nummer]_table[nummer]_counter'
    # -> for a chair: '[global_nummer]_table[nummer]_chair[nummer]_counter'

    models = []
    scenario2(models)
    response = spawn_model_srv.call(models)
    print("successfully spawned model" if response.success else "failed")

def scenario5(models): # obstacles_amount := 31 (obstacles) / 156 (obstale parts) / 165 (obstacle parts + walls + robot (9 extra))
    rospack = rospkg.RosPack()

    table1 = Model()
    table1.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "2_table2_circle_4legs_circle.yaml")
    table1.name = "2_table2_1"
    table1.ns = "namespace1"
    table1.pose.x = 5.0 # no gap
    table1.pose.y = 13.0
    table1.pose.theta = 0
    models.append(table1)

    table2 = Model()
    table2.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "6_table6_circle_1leg_square.yaml")
    table2.name = "6_table6_1"
    table2.ns = "namespace1"
    table2.pose.x = 7.8 # no gap
    table2.pose.y = 13.0
    table2.pose.theta = 0
    models.append(table2)

    table3 = Model()
    table3.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "2_table2_circle_4legs_circle.yaml")
    table3.name = "2_table2_2"
    table3.ns = "namespace1"
    table3.pose.x = 10.6 # no gap
    table3.pose.y = 13.0
    table3.pose.theta = 0
    models.append(table3)

    table4 = Model()
    table4.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "6_table6_circle_1leg_square.yaml")
    table4.name = "6_table6_2"
    table4.ns = "namespace1"
    table4.pose.x = 13.4 # no gap
    table4.pose.y = 13.0
    table4.pose.theta = 0
    models.append(table4)

    table1_chair1 = Model()
    table1_chair1.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "9_chair3_circle_4legs_circle.yaml")
    table1_chair1.name = "9_table2_chair3_1"
    table1_chair1.ns = "namespace1"
    table1_chair1.pose.x = 5.0
    table1_chair1.pose.y = 15.0
    table1_chair1.pose.theta = 0
    models.append(table1_chair1)

    table2_chair1 = Model()
    table2_chair1.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "10_chair4_circle_3legs_circle.yaml")
    table2_chair1.name = "10_table6_chair4_1"
    table2_chair1.ns = "namespace1"
    table2_chair1.pose.x = 7.8
    table2_chair1.pose.y = 15.0
    table2_chair1.pose.theta = 0
    models.append(table2_chair1)

    table1_chair2 = Model()
    table1_chair2.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "9_chair3_circle_4legs_circle.yaml")
    table1_chair2.name = "9_table2_chair3_2"
    table1_chair2.ns = "namespace1"
    table1_chair2.pose.x = 10.6
    table1_chair2.pose.y = 15.0
    table1_chair2.pose.theta = 0
    models.append(table1_chair2)

    table2_chair2 = Model()
    table2_chair2.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "10_chair4_circle_3legs_circle.yaml")
    table2_chair2.name = "10_table6_chair4_2"
    table2_chair2.ns = "namespace1"
    table2_chair2.pose.x = 13.4
    table2_chair2.pose.y = 15.0
    table2_chair2.pose.theta = 0
    models.append(table2_chair2)

    table1_chair3 = Model()
    table1_chair3.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "9_chair3_circle_4legs_circle.yaml")
    table1_chair3.name = "9_table2_chair3_3"
    table1_chair3.ns = "namespace1"
    table1_chair3.pose.x = 5.0
    table1_chair3.pose.y = 11.0
    table1_chair3.pose.theta = math.pi
    models.append(table1_chair3)

    table2_chair3 = Model()
    table2_chair3.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "10_chair4_circle_3legs_circle.yaml")
    table2_chair3.name = "10_table6_chair4_3"
    table2_chair3.ns = "namespace1"
    table2_chair3.pose.x = 7.8
    table2_chair3.pose.y = 11.0
    table2_chair3.pose.theta = math.pi
    models.append(table2_chair3)

    table1_chair4 = Model()
    table1_chair4.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "9_chair3_circle_4legs_circle.yaml")
    table1_chair4.name = "9_table2_chair3_4"
    table1_chair4.ns = "namespace1"
    table1_chair4.pose.x = 10.6
    table1_chair4.pose.y = 11.0
    table1_chair4.pose.theta = math.pi
    models.append(table1_chair4)

    table2_chair4 = Model()
    table2_chair4.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "10_chair4_circle_3legs_circle.yaml")
    table2_chair4.name = "10_table6_chair4_4"
    table2_chair4.ns = "namespace1"
    table2_chair4.pose.x = 13.4
    table2_chair4.pose.y = 11.0
    table2_chair4.pose.theta = math.pi
    models.append(table2_chair4)

    table1_chair5 = Model()
    table1_chair5.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "9_chair3_circle_4legs_circle.yaml")
    table1_chair5.name = "9_table2_chair3_5"
    table1_chair5.ns = "namespace1"
    table1_chair5.pose.x = 3.0
    table1_chair5.pose.y = 13.0
    table1_chair5.pose.theta = math.pi
    models.append(table1_chair5)

    table2_chair5 = Model()
    table2_chair5.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "10_chair4_circle_3legs_circle.yaml")
    table2_chair5.name = "10_table6_chair4_5"
    table2_chair5.ns = "namespace1"
    table2_chair5.pose.x = 15.4
    table2_chair5.pose.y = 13.0
    table2_chair5.pose.theta = math.pi
    models.append(table2_chair5)

    table5_chair1 = Model()
    table5_chair1.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table5_chair1.name = "7_table5_chair1_1"
    table5_chair1.ns = "namespace1"
    table5_chair1.pose.x = 4.0
    table5_chair1.pose.y = 3.0
    table5_chair1.pose.theta = -math.pi/2
    models.append(table5_chair1)

    table5 = Model()
    table5.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "5_table5_square_4legs_square.yaml")
    table5.name = "5_table5_1"
    table5.ns = "namespace1"
    table5.pose.x = 2.75
    table5.pose.y = 3.0
    table5.pose.theta = 0
    models.append(table5)

    table6_chair1 = Model()
    table6_chair1.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table6_chair1.name = "7_table5_chair1_2"
    table6_chair1.ns = "namespace1"
    table6_chair1.pose.x = 5.0
    table6_chair1.pose.y = 4.0
    table6_chair1.pose.theta = math.pi
    models.append(table6_chair1)

    table6 = Model()
    table6.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "5_table5_square_4legs_square.yaml")
    table6.name = "5_table5_2"
    table6.ns = "namespace1"
    table6.pose.x = 5.0
    table6.pose.y = 5.25
    table6.pose.theta = 0
    models.append(table6)

    table7_chair1 = Model()
    table7_chair1.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table7_chair1.name = "7_table5_chair1_3"
    table7_chair1.ns = "namespace1"
    table7_chair1.pose.x = 6.0
    table7_chair1.pose.y = 3.0
    table7_chair1.pose.theta = math.pi/2
    models.append(table7_chair1)

    table7 = Model()
    table7.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "5_table5_square_4legs_square.yaml")
    table7.name = "5_table5_3"
    table7.ns = "namespace1"
    table7.pose.x = 7.25
    table7.pose.y = 3.0
    table7.pose.theta = 0
    models.append(table7)

    table8_chair1 = Model()
    table8_chair1.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table8_chair1.name = "7_table5_chair1_4"
    table8_chair1.ns = "namespace1"
    table8_chair1.pose.x = 5.0
    table8_chair1.pose.y = 2.0
    table8_chair1.pose.theta = 0
    models.append(table8_chair1)

    table8 = Model()
    table8.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "5_table5_square_4legs_square.yaml")
    table8.name = "5_table5_4"
    table8.ns = "namespace1"
    table8.pose.x = 5.0
    table8.pose.y = 0.75
    table8.pose.theta = 0
    models.append(table8)

    table0_chair1 = Model()
    table0_chair1.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "9_chair3_circle_4legs_circle.yaml")
    table0_chair1.name = "9_table0_chair3_1"
    table0_chair1.ns = "namespace1"
    table0_chair1.pose.x = 16.0
    table0_chair1.pose.y = 3.0
    table0_chair1.pose.theta = 0
    models.append(table0_chair1)

    table0_chair2 = Model()
    table0_chair2.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table0_chair2.name = "8_table0_chair2_1"
    table0_chair2.ns = "namespace1"
    table0_chair2.pose.x = 18.0
    table0_chair2.pose.y = 3.0
    table0_chair2.pose.theta = -math.pi/2
    models.append(table0_chair2)

    table0_chair3 = Model()
    table0_chair3.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table0_chair3.name = "8_table0_chair2_2"
    table0_chair3.ns = "namespace1"
    table0_chair3.pose.x = 14.0
    table0_chair3.pose.y = 3
    table0_chair3.pose.theta = math.pi/2
    models.append(table0_chair3)

    table0_chair4 = Model()
    table0_chair4.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table0_chair4.name = "8_table0_chair2_3"
    table0_chair4.ns = "namespace1"
    table0_chair4.pose.x = 16.0
    table0_chair4.pose.y = 5
    table0_chair4.pose.theta = 0
    models.append(table0_chair4)

    table0_chair5 = Model()
    table0_chair5.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table0_chair5.name = "8_table0_chair2_4"
    table0_chair5.ns = "namespace1"
    table0_chair5.pose.x = 16.0
    table0_chair5.pose.y = 1
    table0_chair5.pose.theta = math.pi
    models.append(table0_chair5)

    table0_chair6 = Model()
    table0_chair6.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table0_chair6.name = "8_table0_chair2_5"
    table0_chair6.ns = "namespace1"
    table0_chair6.pose.x = 14.5
    table0_chair6.pose.y = 1.5
    table0_chair6.pose.theta = 3*math.pi/4
    models.append(table0_chair6)

    table0_chair7 = Model()
    table0_chair7.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table0_chair7.name = "8_table0_chair2_6"
    table0_chair7.ns = "namespace1"
    table0_chair7.pose.x = 17.5
    table0_chair7.pose.y = 1.5
    table0_chair7.pose.theta = -3*math.pi/4
    models.append(table0_chair7)

    table0_chair8 = Model()
    table0_chair8.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table0_chair8.name = "8_table0_chair2_7"
    table0_chair8.ns = "namespace1"
    table0_chair8.pose.x = 14.5
    table0_chair8.pose.y = 4.5
    table0_chair8.pose.theta = math.pi/4
    models.append(table0_chair8)

    table0_chair9 = Model()
    table0_chair9.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table0_chair9.name = "8_table0_chair2_8"
    table0_chair9.ns = "namespace1"
    table0_chair9.pose.x = 17.5
    table0_chair9.pose.y = 4.5
    table0_chair9.pose.theta = -math.pi/4
    models.append(table0_chair9)

def scenario4(models): # obstacles_amount := 47 (obstacles) / 246 (obstale parts) / 255 (obstacle parts + walls + robot (9 extra))
    rospack = rospkg.RosPack()

    table1 = Model()
    table1.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "2_table2_circle_4legs_circle.yaml")
    table1.name = "2_table2_1"
    table1.ns = "namespace1"
    table1.pose.x = 5.0 # no gap
    table1.pose.y = 13.0
    table1.pose.theta = 0
    models.append(table1)

    table2 = Model()
    table2.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "2_table2_circle_4legs_circle.yaml")
    table2.name = "2_table2_2"
    table2.ns = "namespace1"
    table2.pose.x = 7.8 # no gap
    table2.pose.y = 13.0
    table2.pose.theta = 0
    models.append(table2)

    table3 = Model()
    table3.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "2_table2_circle_4legs_circle.yaml")
    table3.name = "2_table2_3"
    table3.ns = "namespace1"
    table3.pose.x = 10.6 # no gap
    table3.pose.y = 13.0
    table3.pose.theta = 0
    models.append(table3)

    table4 = Model()
    table4.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "3_table3_square_4legs_circle.yaml")
    table4.name = "3_table3_1"
    table4.ns = "namespace1"
    table4.pose.x = 2.0
    table4.pose.y = 5.0
    table4.pose.theta = 0
    models.append(table4)

    table5 = Model()
    table5.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "3_table3_square_4legs_circle.yaml")
    table5.name = "3_table3_2"
    table5.ns = "namespace1"
    table5.pose.x = 3.6
    table5.pose.y = 5.0
    table5.pose.theta = 0
    models.append(table5)

    table6 = Model()
    table6.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "3_table3_square_4legs_circle.yaml")
    table6.name = "3_table3_3"
    table6.ns = "namespace1"
    table6.pose.x = 5.2
    table6.pose.y = 5.0
    table6.pose.theta = 0
    models.append(table6)

    table7 = Model()
    table7.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "3_table3_square_4legs_circle.yaml")
    table7.name = "3_table3_4"
    table7.ns = "namespace1"
    table7.pose.x = 2.0
    table7.pose.y = 3.4
    table7.pose.theta = 0
    models.append(table7)

    table8 = Model()
    table8.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "3_table3_square_4legs_circle.yaml")
    table8.name = "3_table3_5"
    table8.ns = "namespace1"
    table8.pose.x = 5.2
    table8.pose.y = 3.4
    table8.pose.theta = 0
    models.append(table8)

    table9 = Model()
    table9.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "3_table3_square_4legs_circle.yaml")
    table9.name = "3_table3_6"
    table9.ns = "namespace1"
    table9.pose.x = 2.0
    table9.pose.y = 1.8
    table9.pose.theta = 0
    models.append(table9)

    table10 = Model()
    table10.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "3_table3_square_4legs_circle.yaml")
    table10.name = "3_table3_7"
    table10.ns = "namespace1"
    table10.pose.x = 3.6
    table10.pose.y = 1.8
    table10.pose.theta = 0
    models.append(table10)

    table11 = Model()
    table11.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "3_table3_square_4legs_circle.yaml")
    table11.name = "3_table3_8"
    table11.ns = "namespace1"
    table11.pose.x = 5.2
    table11.pose.y = 1.8
    table11.pose.theta = 0
    models.append(table11)

    table11_chair1 = Model()
    table11_chair1.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "10_chair4_circle_3legs_circle.yaml")
    table11_chair1.name = "10_table3_chair4_1"
    table11_chair1.ns = "namespace1"
    table11_chair1.pose.x = 2.0
    table11_chair1.pose.y = 6.4
    table11_chair1.pose.theta = 0
    models.append(table11_chair1)

    table11_chair2 = Model()
    table11_chair2.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "10_chair4_circle_3legs_circle.yaml")
    table11_chair2.name = "10_table3_chair4_2"
    table11_chair2.ns = "namespace1"
    table11_chair2.pose.x = 3.6
    table11_chair2.pose.y = 6.4
    table11_chair2.pose.theta = 0
    models.append(table11_chair2)

    table11_chair3 = Model()
    table11_chair3.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "10_chair4_circle_3legs_circle.yaml")
    table11_chair3.name = "10_table3_chair4_3"
    table11_chair3.ns = "namespace1"
    table11_chair3.pose.x = 5.2
    table11_chair3.pose.y = 6.4
    table11_chair3.pose.theta = 0
    models.append(table11_chair3)

    table11_chair4 = Model()
    table11_chair4.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "10_chair4_circle_3legs_circle.yaml")
    table11_chair4.name = "10_table3_chair4_4"
    table11_chair4.ns = "namespace1"
    table11_chair4.pose.x = 0.6
    table11_chair4.pose.y = 5.0
    table11_chair4.pose.theta = math.pi/2
    models.append(table11_chair4)

    table11_chair5 = Model()
    table11_chair5.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "10_chair4_circle_3legs_circle.yaml")
    table11_chair5.name = "10_table3_chair4_5"
    table11_chair5.ns = "namespace1"
    table11_chair5.pose.x = 6.6
    table11_chair5.pose.y = 5.0
    table11_chair5.pose.theta = -math.pi/2
    models.append(table11_chair5)

    table11_chair6 = Model()
    table11_chair6.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "10_chair4_circle_3legs_circle.yaml")
    table11_chair6.name = "10_table3_chair4_6"
    table11_chair6.ns = "namespace1"
    table11_chair6.pose.x = 0.6
    table11_chair6.pose.y = 3.4
    table11_chair6.pose.theta = math.pi/2
    models.append(table11_chair6)

    table11_chair7 = Model()
    table11_chair7.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "10_chair4_circle_3legs_circle.yaml")
    table11_chair7.name = "10_table3_chair4_7"
    table11_chair7.ns = "namespace1"
    table11_chair7.pose.x = 6.6
    table11_chair7.pose.y = 3.4
    table11_chair7.pose.theta = -math.pi/2
    models.append(table11_chair7)

    table11_chair8 = Model()
    table11_chair8.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "10_chair4_circle_3legs_circle.yaml")
    table11_chair8.name = "10_table3_chair4_8"
    table11_chair8.ns = "namespace1"
    table11_chair8.pose.x = 0.6
    table11_chair8.pose.y = 1.8
    table11_chair8.pose.theta = math.pi/2
    models.append(table11_chair8)

    table11_chair9 = Model()
    table11_chair9.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "10_chair4_circle_3legs_circle.yaml")
    table11_chair9.name = "10_table3_chair4_9"
    table11_chair9.ns = "namespace1"
    table11_chair9.pose.x = 6.6
    table11_chair9.pose.y = 1.8
    table11_chair9.pose.theta = -math.pi/2
    models.append(table11_chair9)

    table11_chair10 = Model()
    table11_chair10.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "10_chair4_circle_3legs_circle.yaml")
    table11_chair10.name = "10_table3_chair4_10"
    table11_chair10.ns = "namespace1"
    table11_chair10.pose.x = 2.0
    table11_chair10.pose.y = 0.4
    table11_chair10.pose.theta = math.pi
    models.append(table11_chair10)

    table11_chair11 = Model()
    table11_chair11.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "10_chair4_circle_3legs_circle.yaml")
    table11_chair11.name = "10_table3_chair4_11"
    table11_chair11.ns = "namespace1"
    table11_chair11.pose.x = 3.6
    table11_chair11.pose.y = 0.4
    table11_chair11.pose.theta = math.pi
    models.append(table11_chair11)

    table11_chair12 = Model()
    table11_chair12.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "10_chair4_circle_3legs_circle.yaml")
    table11_chair12.name = "10_table3_chair4_12"
    table11_chair12.ns = "namespace1"
    table11_chair12.pose.x = 5.2
    table11_chair12.pose.y = 0.4
    table11_chair12.pose.theta = math.pi
    models.append(table11_chair12)

    table12 = Model()
    table12.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "1_table1_polygon_4legs_circle.yaml")
    table12.name = "1_table1_1"
    table12.ns = "namespace1"
    table12.pose.x = 18.0
    table12.pose.y = 8.0
    table12.pose.theta = math.pi/2
    models.append(table12)

    table12_chair1 = Model()
    table12_chair1.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table12_chair1.name = "7_table1_chair1_1"
    table12_chair1.ns = "namespace1"
    table12_chair1.pose.x = 16.5
    table12_chair1.pose.y = 9.2
    table12_chair1.pose.theta = 0
    models.append(table12_chair1)

    table12_chair2 = Model()
    table12_chair2.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table12_chair2.name = "7_table1_chair1_2"
    table12_chair2.ns = "namespace1"
    table12_chair2.pose.x = 17.5
    table12_chair2.pose.y = 9.2
    table12_chair2.pose.theta = 0
    models.append(table12_chair2)

    table12_chair3 = Model()
    table12_chair3.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table12_chair3.name = "7_table1_chair1_3"
    table12_chair3.ns = "namespace1"
    table12_chair3.pose.x = 18.5
    table12_chair3.pose.y = 9.2
    table12_chair3.pose.theta = 0
    models.append(table12_chair3)

    table12_chair4 = Model()
    table12_chair4.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table12_chair4.name = "7_table1_chair1_4"
    table12_chair4.ns = "namespace1"
    table12_chair4.pose.x = 19.5
    table12_chair4.pose.y = 9.2
    table12_chair4.pose.theta = 0
    models.append(table12_chair4)

    table12_chair5 = Model()
    table12_chair5.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table12_chair5.name = "7_table1_chair1_5"
    table12_chair5.ns = "namespace1"
    table12_chair5.pose.x = 16.5
    table12_chair5.pose.y = 5.0
    table12_chair5.pose.theta = math.pi
    models.append(table12_chair5)

    table12_chair6 = Model()
    table12_chair6.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table12_chair6.name = "7_table1_chair1_6"
    table12_chair6.ns = "namespace1"
    table12_chair6.pose.x = 18.0
    table12_chair6.pose.y = 5.0
    table12_chair6.pose.theta = math.pi
    models.append(table12_chair6)

    table12_chair7 = Model()
    table12_chair7.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table12_chair7.name = "7_table1_chair1_7"
    table12_chair7.ns = "namespace1"
    table12_chair7.pose.x = 19.5
    table12_chair7.pose.y = 5.0
    table12_chair7.pose.theta = math.pi
    models.append(table12_chair7)

    table12_chair8 = Model()
    table12_chair8.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table12_chair8.name = "7_table1_chair1_8"
    table12_chair8.ns = "namespace1"
    table12_chair8.pose.x = 15.75
    table12_chair8.pose.y = 4.0
    table12_chair8.pose.theta = math.pi
    models.append(table12_chair8)

    table12_chair9 = Model()
    table12_chair9.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table12_chair9.name = "7_table1_chair1_9"
    table12_chair9.ns = "namespace1"
    table12_chair9.pose.x = 17.25
    table12_chair9.pose.y = 4.0
    table12_chair9.pose.theta = math.pi
    models.append(table12_chair9)

    table12_chair10 = Model()
    table12_chair10.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table12_chair10.name = "7_table1_chair1_10"
    table12_chair10.ns = "namespace1"
    table12_chair10.pose.x = 18.75
    table12_chair10.pose.y = 4.0
    table12_chair10.pose.theta = math.pi
    models.append(table12_chair10)

    table12_chair11 = Model()
    table12_chair11.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table12_chair11.name = "7_table1_chair1_11"
    table12_chair11.ns = "namespace1"
    table12_chair11.pose.x = 20.25
    table12_chair11.pose.y = 4.0
    table12_chair11.pose.theta = math.pi
    models.append(table12_chair11)

    table12_chair12 = Model()
    table12_chair12.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table12_chair12.name = "7_table1_chair1_12"
    table12_chair12.ns = "namespace1"
    table12_chair12.pose.x = 16.5
    table12_chair12.pose.y = 3.0
    table12_chair12.pose.theta = math.pi
    models.append(table12_chair12)

    table12_chair13 = Model()
    table12_chair13.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table12_chair13.name = "7_table1_chair1_13"
    table12_chair13.ns = "namespace1"
    table12_chair13.pose.x = 18.0
    table12_chair13.pose.y = 3.0
    table12_chair13.pose.theta = math.pi
    models.append(table12_chair13)

    table12_chair14 = Model()
    table12_chair14.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table12_chair14.name = "7_table1_chair1_14"
    table12_chair14.ns = "namespace1"
    table12_chair14.pose.x = 19.5
    table12_chair14.pose.y = 3.0
    table12_chair14.pose.theta = math.pi
    models.append(table12_chair14)

    table12_chair15 = Model()
    table12_chair15.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table12_chair15.name = "7_table1_chair1_15"
    table12_chair15.ns = "namespace1"
    table12_chair15.pose.x = 15.75
    table12_chair15.pose.y = 2.0
    table12_chair15.pose.theta = math.pi
    models.append(table12_chair15)

    table12_chair16 = Model()
    table12_chair16.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table12_chair16.name = "7_table1_chair1_16"
    table12_chair16.ns = "namespace1"
    table12_chair16.pose.x = 17.25
    table12_chair16.pose.y = 2.0
    table12_chair16.pose.theta = math.pi
    models.append(table12_chair16)

    table12_chair17 = Model()
    table12_chair17.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table12_chair17.name = "7_table1_chair1_17"
    table12_chair17.ns = "namespace1"
    table12_chair17.pose.x = 18.75
    table12_chair17.pose.y = 2.0
    table12_chair17.pose.theta = math.pi
    models.append(table12_chair17)

    table12_chair18 = Model()
    table12_chair18.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table12_chair18.name = "7_table1_chair1_18"
    table12_chair18.ns = "namespace1"
    table12_chair18.pose.x = 20.25
    table12_chair18.pose.y = 2.0
    table12_chair18.pose.theta = math.pi
    models.append(table12_chair18)

    table12_chair19 = Model()
    table12_chair19.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table12_chair19.name = "7_table1_chair1_19"
    table12_chair19.ns = "namespace1"
    table12_chair19.pose.x = 15.0
    table12_chair19.pose.y = 0.0
    table12_chair19.pose.theta = math.pi
    models.append(table12_chair19)

    table12_chair20 = Model()
    table12_chair20.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table12_chair20.name = "7_table1_chair1_20"
    table12_chair20.ns = "namespace1"
    table12_chair20.pose.x = 16.5
    table12_chair20.pose.y = 0.0
    table12_chair20.pose.theta = math.pi
    models.append(table12_chair20)

    table12_chair21 = Model()
    table12_chair21.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table12_chair21.name = "7_table1_chair1_21"
    table12_chair21.ns = "namespace1"
    table12_chair21.pose.x = 18.0
    table12_chair21.pose.y = 0.0
    table12_chair21.pose.theta = math.pi
    models.append(table12_chair21)

    table12_chair22 = Model()
    table12_chair22.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table12_chair22.name = "7_table1_chair1_22"
    table12_chair22.ns = "namespace1"
    table12_chair22.pose.x = 19.5
    table12_chair22.pose.y = 0.0
    table12_chair22.pose.theta = math.pi
    models.append(table12_chair22)

    table12_chair23 = Model()
    table12_chair23.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table12_chair23.name = "7_table1_chair1_23"
    table12_chair23.ns = "namespace1"
    table12_chair23.pose.x = 21.0
    table12_chair23.pose.y = 0.0
    table12_chair23.pose.theta = math.pi
    models.append(table12_chair23)

def scenario3(models): # obstacles_amount := 35 (obstacles) / 194 (obstale parts) / 203 (obstacle parts + walls + robot (9 extra))
    rospack = rospkg.RosPack()

    # conference room
    # 5 rectangle tables next to each other as one really big table with two edges
    table1 = Model()
    table1.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "4_table4_polygon_4legs_square.yaml")
    table1.name = "4_table4_1"
    table1.ns = "namespace1"
    table1.pose.x = 2.20 # no gap
    table1.pose.y = 11.80
    table1.pose.theta = 0
    models.append(table1)

    table2 = Model()
    table2.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "4_table4_polygon_4legs_square.yaml")
    table2.name = "4_table4_2"
    table2.ns = "namespace1"
    table2.pose.x = 15.80 # no gap
    table2.pose.y = 11.80
    table2.pose.theta = 0
    models.append(table2)

    table3 = Model()
    table3.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "4_table4_polygon_4legs_square.yaml")
    table3.name = "4_table4_3"
    table3.ns = "namespace1"
    table3.pose.x = 5.0
    table3.pose.y = 13.0
    table3.pose.theta = math.pi/2
    models.append(table3)

    table4 = Model()
    table4.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "4_table4_polygon_4legs_square.yaml")
    table4.name = "4_table4_4"
    table4.ns = "namespace1"
    table4.pose.x = 9.0 # no gap
    table4.pose.y = 13.0
    table4.pose.theta = math.pi/2
    models.append(table4)

    table5 = Model()
    table5.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "4_table4_polygon_4legs_square.yaml")
    table5.name = "4_table4_5"
    table5.ns = "namespace1"
    table5.pose.x = 13.0 # gap vs. no gap
    table5.pose.y = 13.0
    table5.pose.theta = math.pi/2
    models.append(table5)

    table5_chair1 = Model()
    table5_chair1.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table5_chair1.name = "7_table4_chair1_1"
    table5_chair1.ns = "namespace1"
    table5_chair1.pose.x = 17.0
    table5_chair1.pose.y = 12.5
    table5_chair1.pose.theta = -math.pi/2
    models.append(table5_chair1)

    table5_chair2 = Model()
    table5_chair2.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table5_chair2.name = "7_table4_chair1_2"
    table5_chair2.ns = "namespace1"
    table5_chair2.pose.x = 1.0
    table5_chair2.pose.y = 12.5
    table5_chair2.pose.theta = math.pi/2
    models.append(table5_chair2)

    table5_chair3 = Model()
    table5_chair3.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table5_chair3.name = "7_table4_chair1_3"
    table5_chair3.ns = "namespace1"
    table5_chair3.pose.x = 4.25
    table5_chair3.pose.y = 14.25
    table5_chair3.pose.theta = 0
    models.append(table5_chair3)

    table5_chair4 = Model()
    table5_chair4.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table5_chair4.name = "7_table4_chair1_4"
    table5_chair4.ns = "namespace1"
    table5_chair4.pose.x = 5.75
    table5_chair4.pose.y = 14.25
    table5_chair4.pose.theta = 0
    models.append(table5_chair4)

    table5_chair5 = Model()
    table5_chair5.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table5_chair5.name = "7_table4_chair1_5"
    table5_chair5.ns = "namespace1"
    table5_chair5.pose.x = 8.35
    table5_chair5.pose.y = 14.25
    table5_chair5.pose.theta = 0
    models.append(table5_chair5)

    table5_chair6 = Model()
    table5_chair6.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table5_chair6.name = "7_table4_chair1_6"
    table5_chair6.ns = "namespace1"
    table5_chair6.pose.x = 9.85
    table5_chair6.pose.y = 14.25
    table5_chair6.pose.theta = 0
    models.append(table5_chair6)

    table5_chair7 = Model()
    table5_chair7.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table5_chair7.name = "7_table4_chair1_7"
    table5_chair7.ns = "namespace1"
    table5_chair7.pose.x = 12.45
    table5_chair7.pose.y = 14.25
    table5_chair7.pose.theta = 0
    models.append(table5_chair7)

    table5_chair8 = Model()
    table5_chair8.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table5_chair8.name = "7_table4_chair1_8"
    table5_chair8.ns = "namespace1"
    table5_chair8.pose.x = 13.95
    table5_chair8.pose.y = 14.25
    table5_chair8.pose.theta = 0
    models.append(table5_chair8)

    table5_chair9 = Model()
    table5_chair9.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table5_chair9.name = "7_table4_chair1_0"
    table5_chair9.ns = "namespace1"
    table5_chair9.pose.x = 17.0
    table5_chair9.pose.y = 11.0
    table5_chair9.pose.theta = -math.pi/2
    models.append(table5_chair9)

    table5_chair10 = Model()
    table5_chair10.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table5_chair10.name = "7_table4_chair1_10"
    table5_chair10.ns = "namespace1"
    table5_chair10.pose.x = 1.0
    table5_chair10.pose.y = 11.0
    table5_chair10.pose.theta = math.pi/2
    models.append(table5_chair10)

    table6 = Model()
    table6.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "4_table4_polygon_4legs_square.yaml")
    table6.name = "4_table4_6"
    table6.ns = "namespace1"
    table6.pose.x = 9.0
    table6.pose.y = 7.5
    table6.pose.theta = math.pi/2
    models.append(table6)

    table6_chair1 = Model()
    table6_chair1.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table6_chair1.name = "7_table4_chair1_11"
    table6_chair1.ns = "namespace1"
    table6_chair1.pose.x = 9.0
    table6_chair1.pose.y = 6.25
    table6_chair1.pose.theta = math.pi
    models.append(table6_chair1)

    # for square tables next to each other, forming a bigger square
    table7 = Model()
    table7.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "5_table5_square_4legs_square.yaml")
    table7.name = "5_table5_1"
    table7.ns = "namespace1"
    table7.pose.x = 17.6
    table7.pose.y = 2.0
    table7.pose.theta = 0
    models.append(table7)

    table8 = Model()
    table8.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "5_table5_square_4legs_square.yaml")
    table8.name = "5_table5_2"
    table8.ns = "namespace1"
    table8.pose.x = 16.0
    table8.pose.y = 2.0
    table8.pose.theta = 0
    models.append(table8)

    table9 = Model()
    table9.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "5_table5_square_4legs_square.yaml")
    table9.name = "5_table5_3"
    table9.ns = "namespace1"
    table9.pose.x = 17.6
    table9.pose.y = 0.4
    table9.pose.theta = 0
    models.append(table9)

    table10 = Model()
    table10.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "5_table5_square_4legs_square.yaml")
    table10.name = "5_table5_4"
    table10.ns = "namespace1"
    table10.pose.x = 16.0
    table10.pose.y = 0.4
    table10.pose.theta = 0
    models.append(table10)

    table10_chair1 = Model()
    table10_chair1.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table10_chair1.name = "8_table5_chair2_1"
    table10_chair1.ns = "namespace1"
    table10_chair1.pose.x = 17.6
    table10_chair1.pose.y = -0.825
    table10_chair1.pose.theta = math.pi
    models.append(table10_chair1)

    table10_chair2 = Model()
    table10_chair2.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table10_chair2.name = "8_table5_chair2_2"
    table10_chair2.ns = "namespace1"
    table10_chair2.pose.x = 16.0
    table10_chair2.pose.y = -0.825
    table10_chair2.pose.theta = math.pi
    models.append(table10_chair2)

    table10_chair3 = Model()
    table10_chair3.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table10_chair3.name = "8_table5_chair2_3"
    table10_chair3.ns = "namespace1"
    table10_chair3.pose.x = 17.6
    table10_chair3.pose.y = 3.225
    table10_chair3.pose.theta = 0
    models.append(table10_chair3)

    table10_chair4 = Model()
    table10_chair4.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table10_chair4.name = "8_table5_chair2_4"
    table10_chair4.ns = "namespace1"
    table10_chair4.pose.x = 16.0
    table10_chair4.pose.y = 3.225
    table10_chair4.pose.theta = 0
    models.append(table10_chair4)

    table10_chair5 = Model()
    table10_chair5.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table10_chair5.name = "8_table5_chair2_5"
    table10_chair5.ns = "namespace1"
    table10_chair5.pose.x = 18.825
    table10_chair5.pose.y = 0.4
    table10_chair5.pose.theta = -math.pi/2
    models.append(table10_chair5)

    table10_chair6 = Model()
    table10_chair6.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table10_chair6.name = "8_table5_chair2_6"
    table10_chair6.ns = "namespace1"
    table10_chair6.pose.x = 14.775
    table10_chair6.pose.y = 0.4
    table10_chair6.pose.theta = math.pi/2
    models.append(table10_chair6)

    table10_chair7 = Model()
    table10_chair7.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table10_chair7.name = "8_table5_chair2_7"
    table10_chair7.ns = "namespace1"
    table10_chair7.pose.x = 18.825
    table10_chair7.pose.y = 2.0
    table10_chair7.pose.theta = -math.pi/2
    models.append(table10_chair7)

    table10_chair8 = Model()
    table10_chair8.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table10_chair8.name = "8_table5_chair2_8"
    table10_chair8.ns = "namespace1"
    table10_chair8.pose.x = 14.775
    table10_chair8.pose.y = 2.0
    table10_chair8.pose.theta = math.pi/2
    models.append(table10_chair8)

    # three square tables near each other, forming a triangle
    table11 = Model()
    table11.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "3_table3_square_4legs_circle.yaml")
    table11.name = "3_table3_1"
    table11.ns = "namespace1"
    table11.pose.x = 3.1
    table11.pose.y = 0.1
    table11.pose.theta = 0
    models.append(table11)

    table11_chair1 = Model()
    table11_chair1.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "9_chair3_circle_4legs_circle.yaml")
    table11_chair1.name = "9_table3_chair3_1"
    table11_chair1.ns = "namespace1"
    table11_chair1.pose.x = 3.1
    table11_chair1.pose.y = -1.375
    table11_chair1.pose.theta = 0
    models.append(table11_chair1)

    table12 = Model()
    table12.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "3_table3_square_4legs_circle.yaml")
    table12.name = "3_table3_2"
    table12.ns = "namespace1"
    table12.pose.x = 4.2
    table12.pose.y = 2.0
    table12.pose.theta = 0.85*math.pi/5
    models.append(table12)

    table12_chair1 = Model()
    table12_chair1.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "9_chair3_circle_4legs_circle.yaml")
    table12_chair1.name = "9_table3_chair3_2"
    table12_chair1.ns = "namespace1"
    table12_chair1.pose.x = 5.5
    table12_chair1.pose.y = 2.75
    table12_chair1.pose.theta = 0.85*math.pi/5
    models.append(table12_chair1)

    table13 = Model()
    table13.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "3_table3_square_4legs_circle.yaml")
    table13.name = "3_table3_3"
    table13.ns = "namespace1"
    table13.pose.x = 2
    table13.pose.y = 2.0
    table13.pose.theta = -0.85*math.pi/5
    models.append(table13)

    table13_chair1 = Model()
    table13_chair1.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "9_chair3_circle_4legs_circle.yaml")
    table13_chair1.name = "9_table3_chair3_3"
    table13_chair1.ns = "namespace1"
    table13_chair1.pose.x = 0.7
    table13_chair1.pose.y = 2.75
    table13_chair1.pose.theta = -0.85*math.pi/5
    models.append(table13_chair1)

def scenario2(models): # obstacles_amount := 44 (obstacles) / 244 (obstale parts) / 253 (obstacle parts + walls + robot (9 extra))
    rospack = rospkg.RosPack()

    # a table for a tv for example in the middle of the room, without chairs
    table1 = Model()
    table1.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "5_table5_square_4legs_square.yaml")
    table1.name = "5_table5_1"
    table1.ns = "namespace1"
    table1.pose.x = 8.0
    table1.pose.y = 6.0
    table1.pose.theta = math.pi/4
    models.append(table1)

    # a round table with 10 rectangle chairs really near to each other
    table2 = Model()
    table2.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "2_table2_circle_4legs_circle.yaml")
    table2.name = "2_table2_1"
    table2.ns = "namespace1"
    table2.pose.x = 18.0
    table2.pose.y = 3.0
    table2.pose.theta = 0
    models.append(table2)

    table2_chair1 = Model()
    table2_chair1.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table2_chair1.name = "7_table2_chair1_1"
    table2_chair1.ns = "namespace1"
    table2_chair1.pose.x = 18.0
    table2_chair1.pose.y = 4.75 # 5
    table2_chair1.pose.theta = 0
    models.append(table2_chair1)

    table2_chair2 = Model()
    table2_chair2.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table2_chair2.name = "7_table2_chair1_2"
    table2_chair2.ns = "namespace1"
    table2_chair2.pose.x = 18.0
    table2_chair2.pose.y = 1.25 # 1
    table2_chair2.pose.theta = math.pi
    models.append(table2_chair2)

    table2_chair3 = Model()
    table2_chair3.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table2_chair3.name = "7_table2_chair1_3"
    table2_chair3.ns = "namespace1"
    table2_chair3.pose.x = 16.95 # 16.5
    table2_chair3.pose.y = 1.55 # 1.5
    table2_chair3.pose.theta = 3.25*math.pi/4 # 3*math.pi/4
    models.append(table2_chair3)

    table2_chair4 = Model()
    table2_chair4.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table2_chair4.name = "7_table2_chair1_4"
    table2_chair4.ns = "namespace1"
    table2_chair4.pose.x = 19.05 # 19.5
    table2_chair4.pose.y = 1.55 # 1.5
    table2_chair4.pose.theta = -3.25*math.pi/4 # -3*math.pi/4
    models.append(table2_chair4)

    table2_chair5 = Model()
    table2_chair5.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table2_chair5.name = "7_table2_chair1_5"
    table2_chair5.ns = "namespace1"
    table2_chair5.pose.x = 16.95 # 16.5
    table2_chair5.pose.y = 4.45 # 4.5
    table2_chair5.pose.theta = 0.75*math.pi/4 # math.pi/4
    models.append(table2_chair5)

    table2_chair6 = Model()
    table2_chair6.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table2_chair6.name = "7_table2_chair1_6"
    table2_chair6.ns = "namespace1"
    table2_chair6.pose.x = 19.05 # 19.5
    table2_chair6.pose.y = 4.45 # 4.5
    table2_chair6.pose.theta = -0.75*math.pi/4 # -math.pi/4
    models.append(table2_chair6)

    table2_chair7 = Model()
    table2_chair7.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table2_chair7.name = "7_table2_chair1_7"
    table2_chair7.ns = "namespace1"
    table2_chair7.pose.x = 19.7 # 20
    table2_chair7.pose.y = 3.55 # 3
    table2_chair7.pose.theta = -0.8*math.pi/2 # -math.pi/2
    models.append(table2_chair7)

    table2_chair8 = Model()
    table2_chair8.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table2_chair8.name = "7_table2_chair1_8"
    table2_chair8.ns = "namespace1"
    table2_chair8.pose.x = 16.3 # 16
    table2_chair8.pose.y = 3.55 # 3
    table2_chair8.pose.theta = 0.8*math.pi/2 # math.pi/2
    models.append(table2_chair8)

    table2_chair9 = Model()
    table2_chair9.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table2_chair9.name = "7_table2_chair1_9"
    table2_chair9.ns = "namespace1"
    table2_chair9.pose.x = 19.675 # 20
    table2_chair9.pose.y = 2.45 # 3
    table2_chair9.pose.theta = -1.2*math.pi/2 # -math.pi/2
    models.append(table2_chair9)

    table2_chair10 = Model()
    table2_chair10.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table2_chair10.name = "7_table2_chair1_10"
    table2_chair10.ns = "namespace1"
    table2_chair10.pose.x = 16.325 # 16
    table2_chair10.pose.y = 2.45 # 3
    table2_chair10.pose.theta = 1.2*math.pi/2 # math.pi/2
    models.append(table2_chair10)

    # 3 rectangle tables next to each other as one really big table (highter difficulty level, since the 4 legs could look like a chair, but it will be just two tables really near)
    table3 = Model()
    table3.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "4_table4_polygon_4legs_square.yaml")
    table3.name = "4_table4_1"
    table3.ns = "namespace1"
    table3.pose.x = 2.0
    table3.pose.y = 13.0
    table3.pose.theta = math.pi/2
    models.append(table3)

    table4 = Model()
    table4.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "4_table4_polygon_4legs_square.yaml")
    table4.name = "4_table4_2"
    table4.ns = "namespace1"
    table4.pose.x = 6.1 # 6.1 to have a small gap, 6 for no gap between the tables
    table4.pose.y = 13.0
    table4.pose.theta = math.pi/2
    models.append(table4)

    table5 = Model()
    table5.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "4_table4_polygon_4legs_square.yaml")
    table5.name = "4_table4_3"
    table5.ns = "namespace1"
    table5.pose.x = 10.2 # gap vs. no gap
    table5.pose.y = 13.0
    table5.pose.theta = math.pi/2
    models.append(table5)

    table5_chair1 = Model()
    table5_chair1.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table5_chair1.name = "8_table4_chair2_1"
    table5_chair1.ns = "namespace1"
    table5_chair1.pose.x = 12.625
    table5_chair1.pose.y = 13.0
    table5_chair1.pose.theta = -math.pi/2
    models.append(table5_chair1)

    table5_chair2 = Model()
    table5_chair2.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table5_chair2.name = "8_table4_chair2_2"
    table5_chair2.ns = "namespace1"
    table5_chair2.pose.x = -0.425
    table5_chair2.pose.y = 13.0
    table5_chair2.pose.theta = math.pi/2
    models.append(table5_chair2)

    table5_chair3 = Model()
    table5_chair3.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table5_chair3.name = "8_table4_chair2_3"
    table5_chair3.ns = "namespace1"
    table5_chair3.pose.x = 0.75
    table5_chair3.pose.y = 14.25
    table5_chair3.pose.theta = 0
    models.append(table5_chair3)

    table5_chair4 = Model()
    table5_chair4.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table5_chair4.name = "8_table4_chair2_4"
    table5_chair4.ns = "namespace1"
    table5_chair4.pose.x = 2.0
    table5_chair4.pose.y = 14.25
    table5_chair4.pose.theta = 0
    models.append(table5_chair4)

    table5_chair5 = Model()
    table5_chair5.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table5_chair5.name = "8_table4_chair2_5"
    table5_chair5.ns = "namespace1"
    table5_chair5.pose.x = 3.25
    table5_chair5.pose.y = 14.25
    table5_chair5.pose.theta = 0
    models.append(table5_chair5)

    table5_chair6 = Model()
    table5_chair6.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table5_chair6.name = "8_table4_chair2_6"
    table5_chair6.ns = "namespace1"
    table5_chair6.pose.x = 4.85
    table5_chair6.pose.y = 14.25
    table5_chair6.pose.theta = 0
    models.append(table5_chair6)

    table5_chair7 = Model()
    table5_chair7.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table5_chair7.name = "8_table4_chair2_7"
    table5_chair7.ns = "namespace1"
    table5_chair7.pose.x = 6.1
    table5_chair7.pose.y = 14.25
    table5_chair7.pose.theta = 0
    models.append(table5_chair7)

    table5_chair8 = Model()
    table5_chair8.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table5_chair8.name = "8_table4_chair2_8"
    table5_chair8.ns = "namespace1"
    table5_chair8.pose.x = 7.35
    table5_chair8.pose.y = 14.25
    table5_chair8.pose.theta = 0
    models.append(table5_chair8)

    table5_chair9 = Model()
    table5_chair9.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table5_chair9.name = "8_table4_chair2_9"
    table5_chair9.ns = "namespace1"
    table5_chair9.pose.x = 8.95
    table5_chair9.pose.y = 14.25
    table5_chair9.pose.theta = 0
    models.append(table5_chair9)

    table5_chair10 = Model()
    table5_chair10.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table5_chair10.name = "8_table4_chair2_10"
    table5_chair10.ns = "namespace1"
    table5_chair10.pose.x = 10.2
    table5_chair10.pose.y = 14.25
    table5_chair10.pose.theta = 0
    models.append(table5_chair10)

    table5_chair11 = Model()
    table5_chair11.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table5_chair11.name = "8_table4_chair2_11"
    table5_chair11.ns = "namespace1"
    table5_chair11.pose.x = 11.45
    table5_chair11.pose.y = 14.25
    table5_chair11.pose.theta = 0
    models.append(table5_chair11)

    table5_chair12 = Model()
    table5_chair12.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table5_chair12.name = "8_table4_chair2_12"
    table5_chair12.ns = "namespace1"
    table5_chair12.pose.x = 0.75
    table5_chair12.pose.y = 11.75
    table5_chair12.pose.theta = math.pi
    models.append(table5_chair12)

    table5_chair13 = Model()
    table5_chair13.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table5_chair13.name = "8_table4_chair2_13"
    table5_chair13.ns = "namespace1"
    table5_chair13.pose.x = 2.0
    table5_chair13.pose.y = 11.75
    table5_chair13.pose.theta = math.pi
    models.append(table5_chair13)

    table5_chair14 = Model()
    table5_chair14.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table5_chair14.name = "8_table4_chair2_14"
    table5_chair14.ns = "namespace1"
    table5_chair14.pose.x = 3.25
    table5_chair14.pose.y = 11.75
    table5_chair14.pose.theta = math.pi
    models.append(table5_chair14)

    table5_chair15 = Model()
    table5_chair15.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table5_chair15.name = "8_table4_chair2_15"
    table5_chair15.ns = "namespace1"
    table5_chair15.pose.x = 4.85
    table5_chair15.pose.y = 11.75
    table5_chair15.pose.theta = math.pi
    models.append(table5_chair15)

    table5_chair16 = Model()
    table5_chair16.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table5_chair16.name = "8_table4_chair2_16"
    table5_chair16.ns = "namespace1"
    table5_chair16.pose.x = 6.1
    table5_chair16.pose.y = 11.75
    table5_chair16.pose.theta = math.pi
    models.append(table5_chair16)

    table5_chair17 = Model()
    table5_chair17.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table5_chair17.name = "8_table4_chair2_17"
    table5_chair17.ns = "namespace1"
    table5_chair17.pose.x = 7.35
    table5_chair17.pose.y = 11.75
    table5_chair17.pose.theta = math.pi
    models.append(table5_chair17)

    table5_chair18 = Model()
    table5_chair18.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table5_chair18.name = "8_table4_chair2_18"
    table5_chair18.ns = "namespace1"
    table5_chair18.pose.x = 8.95
    table5_chair18.pose.y = 11.75
    table5_chair18.pose.theta = math.pi
    models.append(table5_chair18)

    table5_chair19 = Model()
    table5_chair19.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table5_chair19.name = "8_table4_chair2_19"
    table5_chair19.ns = "namespace1"
    table5_chair19.pose.x = 10.2
    table5_chair19.pose.y = 11.75
    table5_chair19.pose.theta = math.pi
    models.append(table5_chair19)

    table5_chair20 = Model()
    table5_chair20.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table5_chair20.name = "8_table4_chair2_20"
    table5_chair20.ns = "namespace1"
    table5_chair20.pose.x = 11.45
    table5_chair20.pose.y = 11.75
    table5_chair20.pose.theta = math.pi
    models.append(table5_chair20)

    # 2 round tables next to each other with round chairs on each side except for one
    table6 = Model()
    table6.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "6_table6_circle_1leg_square.yaml")
    table6.name = "6_table6_1"
    table6.ns = "namespace1"
    table6.pose.x = 3.0
    table6.pose.y = 4.0
    table6.pose.theta = 0
    models.append(table6)

    table7 = Model()
    table7.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "6_table6_circle_1leg_square.yaml")
    table7.name = "6_table6_2"
    table7.ns = "namespace1"
    table7.pose.x = 5.0
    table7.pose.y = 2.0
    table7.pose.theta = 0
    models.append(table7)

    table7_chair1 = Model()
    table7_chair1.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "9_chair3_circle_4legs_circle.yaml")
    table7_chair1.name = "9_table6_chair3_1"
    table7_chair1.ns = "namespace1"
    table7_chair1.pose.x = 2.875
    table7_chair1.pose.y = 1.875
    table7_chair1.pose.theta = 0 # math.pi/4 vs. 0!? # TODO: somehow count as two obstacles!? the problem is in the rotation?!
    models.append(table7_chair1)

    table7_chair2 = Model()
    table7_chair2.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "9_chair3_circle_4legs_circle.yaml")
    table7_chair2.name = "9_table6_chair3_2"
    table7_chair2.ns = "namespace1"
    table7_chair2.pose.x = 3.5
    table7_chair2.pose.y = 0.5
    table7_chair2.pose.theta = math.pi/4
    models.append(table7_chair2)

    table7_chair3 = Model()
    table7_chair3.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "9_chair3_circle_4legs_circle.yaml")
    table7_chair3.name = "9_table6_chair3_3"
    table7_chair3.ns = "namespace1"
    table7_chair3.pose.x = 4.75
    table7_chair3.pose.y = -0.125
    table7_chair3.pose.theta = math.pi/4
    models.append(table7_chair3)

    table7_chair4 = Model()
    table7_chair4.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "9_chair3_circle_4legs_circle.yaml")
    table7_chair4.name = "9_table6_chair3_4"
    table7_chair4.ns = "namespace1"
    table7_chair4.pose.x = 6.125
    table7_chair4.pose.y = 0.25
    table7_chair4.pose.theta = math.pi/4
    models.append(table7_chair4)

    table7_chair5 = Model()
    table7_chair5.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "9_chair3_circle_4legs_circle.yaml")
    table7_chair5.name = "9_table6_chair3_5"
    table7_chair5.ns = "namespace1"
    table7_chair5.pose.x = 1.5
    table7_chair5.pose.y = 2.5
    table7_chair5.pose.theta = math.pi/4
    models.append(table7_chair5)

    table7_chair6 = Model()
    table7_chair6.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "9_chair3_circle_4legs_circle.yaml")
    table7_chair6.name = "9_table6_chair3_6"
    table7_chair6.ns = "namespace1"
    table7_chair6.pose.x = 0.875
    table7_chair6.pose.y = 3.75
    table7_chair6.pose.theta = math.pi/4
    models.append(table7_chair6)

    table7_chair7 = Model()
    table7_chair7.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "9_chair3_circle_4legs_circle.yaml")
    table7_chair7.name = "9_table6_chair3_7"
    table7_chair7.ns = "namespace1"
    table7_chair7.pose.x = 1.125
    table7_chair7.pose.y = 5.125
    table7_chair7.pose.theta = math.pi/4
    models.append(table7_chair7)

def scenario1(models): # obstacles_amount := 26 (obstacles) / 133 (obstale parts) / 142 (obstacle parts + walls + robot (9 extra))
    rospack = rospkg.RosPack()

    table1 = Model()
    table1.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "1_table1_polygon_4legs_circle.yaml")
    table1.name = "1_table1_1"
    table1.ns = "namespace1"
    table1.pose.x = 10.0
    table1.pose.y = 3.0
    table1.pose.theta = 0
    models.append(table1)

    table1_chair1 = Model()
    table1_chair1.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table1_chair1.name = "7_table1_chair1_1"
    table1_chair1.ns = "namespace1"
    table1_chair1.pose.x = 11.5
    table1_chair1.pose.y = 3.0
    table1_chair1.pose.theta = -math.pi/2
    models.append(table1_chair1)

    table1_chair2 = Model()
    table1_chair2.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table1_chair2.name = "7_table1_chair1_2"
    table1_chair2.ns = "namespace1"
    table1_chair2.pose.x = 11.5
    table1_chair2.pose.y = 4.25
    table1_chair2.pose.theta = -math.pi/2
    models.append(table1_chair2)

    table1_chair3 = Model()
    table1_chair3.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table1_chair3.name = "7_table1_chair1_3"
    table1_chair3.ns = "namespace1"
    table1_chair3.pose.x = 11.5
    table1_chair3.pose.y = 1.75
    table1_chair3.pose.theta = -math.pi/2
    models.append(table1_chair3)

    table1_chair4 = Model()
    table1_chair4.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table1_chair4.name = "7_table1_chair1_4"
    table1_chair4.ns = "namespace1"
    table1_chair4.pose.x = 8.5
    table1_chair4.pose.y = 3.0
    table1_chair4.pose.theta = math.pi/2
    models.append(table1_chair4)

    table1_chair5 = Model()
    table1_chair5.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table1_chair5.name = "7_table1_chair1_5"
    table1_chair5.ns = "namespace1"
    table1_chair5.pose.x = 8.5
    table1_chair5.pose.y = 4.25
    table1_chair5.pose.theta = math.pi/2
    models.append(table1_chair5)

    table1_chair6 = Model()
    table1_chair6.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table1_chair6.name = "7_table1_chair1_6"
    table1_chair6.ns = "namespace1"
    table1_chair6.pose.x = 8.5
    table1_chair6.pose.y = 1.75
    table1_chair6.pose.theta = math.pi/2
    models.append(table1_chair6)

    table1_chair7 = Model()
    table1_chair7.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table1_chair7.name = "7_table1_chair1_7"
    table1_chair7.ns = "namespace1"
    table1_chair7.pose.x = 10.0
    table1_chair7.pose.y = 5.5
    table1_chair7.pose.theta = 0
    models.append(table1_chair7)

    table1_chair8 = Model()
    table1_chair8.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table1_chair8.name = "7_table1_chair1_8"
    table1_chair8.ns = "namespace1"
    table1_chair8.pose.x = 10.0
    table1_chair8.pose.y = 0.5
    table1_chair8.pose.theta = math.pi
    models.append(table1_chair8)

    table2 = Model()
    table2.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "2_table2_circle_4legs_circle.yaml")
    table2.name = "2_table2_1"
    table2.ns = "namespace1"
    table2.pose.x = 20.0
    table2.pose.y = 3.0
    table2.pose.theta = 0
    models.append(table2)

    table2_chair1 = Model()
    table2_chair1.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "10_chair4_circle_3legs_circle.yaml")
    table2_chair1.name = "10_table2_chair4_1"
    table2_chair1.ns = "namespace1"
    table2_chair1.pose.x = 22.0
    table2_chair1.pose.y = 3
    table2_chair1.pose.theta = -math.pi/2
    models.append(table2_chair1)

    table2_chair2 = Model()
    table2_chair2.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "10_chair4_circle_3legs_circle.yaml")
    table2_chair2.name = "10_table2_chair4_2"
    table2_chair2.ns = "namespace1"
    table2_chair2.pose.x = 18.0
    table2_chair2.pose.y = 3
    table2_chair2.pose.theta = math.pi/2
    models.append(table2_chair2)

    table2_chair3 = Model()
    table2_chair3.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "10_chair4_circle_3legs_circle.yaml")
    table2_chair3.name = "10_table2_chair4_3"
    table2_chair3.ns = "namespace1"
    table2_chair3.pose.x = 20.0
    table2_chair3.pose.y = 5
    table2_chair3.pose.theta = 0
    models.append(table2_chair3)

    table2_chair4 = Model()
    table2_chair4.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "10_chair4_circle_3legs_circle.yaml")
    table2_chair4.name = "10_table2_chair4_4"
    table2_chair4.ns = "namespace1"
    table2_chair4.pose.x = 20.0
    table2_chair4.pose.y = 1
    table2_chair4.pose.theta = math.pi
    models.append(table2_chair4)

    table2_chair5 = Model()
    table2_chair5.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "10_chair4_circle_3legs_circle.yaml")
    table2_chair5.name = "10_table2_chair4_5"
    table2_chair5.ns = "namespace1"
    table2_chair5.pose.x = 18.5
    table2_chair5.pose.y = 1.5
    table2_chair5.pose.theta = 3*math.pi/4
    models.append(table2_chair5)

    table2_chair6 = Model()
    table2_chair6.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "10_chair4_circle_3legs_circle.yaml")
    table2_chair6.name = "10_table2_chair4_6"
    table2_chair6.ns = "namespace1"
    table2_chair6.pose.x = 21.5
    table2_chair6.pose.y = 1.5
    table2_chair6.pose.theta = -3*math.pi/4
    models.append(table2_chair6)

    table2_chair7 = Model()
    table2_chair7.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "10_chair4_circle_3legs_circle.yaml")
    table2_chair7.name = "10_table2_chair4_7"
    table2_chair7.ns = "namespace1"
    table2_chair7.pose.x = 18.5
    table2_chair7.pose.y = 4.5
    table2_chair7.pose.theta = math.pi/4
    models.append(table2_chair7)

    table2_chair8 = Model()
    table2_chair8.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "10_chair4_circle_3legs_circle.yaml")
    table2_chair8.name = "10_table2_chair4_8"
    table2_chair8.ns = "namespace1"
    table2_chair8.pose.x = 21.5
    table2_chair8.pose.y = 4.5
    table2_chair8.pose.theta = -math.pi/4
    models.append(table2_chair8)

    table3 = Model()
    table3.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "6_table6_circle_1leg_square.yaml")
    table3.name = "6_table6_1"
    table3.ns = "namespace1"
    table3.pose.x = 20.0
    table3.pose.y = 13.0
    table3.pose.theta = 0
    models.append(table3)

    table3_chair1 = Model()
    table3_chair1.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table3_chair1.name = "7_table6_chair1_1"
    table3_chair1.ns = "namespace1"
    table3_chair1.pose.x = 22.0
    table3_chair1.pose.y = 13.0
    table3_chair1.pose.theta = -math.pi/2
    models.append(table3_chair1)

    table3_chair2 = Model()
    table3_chair2.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "7_chair1_rectangle_4legs_rectangle.yaml")
    table3_chair2.name = "7_table6_chair1_2"
    table3_chair2.ns = "namespace1"
    table3_chair2.pose.x = 18.0
    table3_chair2.pose.y = 13.0
    table3_chair2.pose.theta = math.pi/2
    models.append(table3_chair2)

    table4 = Model()
    table4.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "5_table5_square_4legs_square.yaml")
    table4.name = "5_table5_1"
    table4.ns = "namespace1"
    table4.pose.x = 10.0
    table4.pose.y = 13.0
    table4.pose.theta = math.pi/4
    models.append(table4)

    table4_chair1 = Model()
    table4_chair1.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table4_chair1.name = "8_table5_chair2_1"
    table4_chair1.ns = "namespace1"
    table4_chair1.pose.x = 11.0
    table4_chair1.pose.y = 14.0
    table4_chair1.pose.theta = -math.pi/4
    models.append(table4_chair1)

    table4_chair2 = Model()
    table4_chair2.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table4_chair2.name = "8_table5_chair2_2"
    table4_chair2.ns = "namespace1"
    table4_chair2.pose.x = 9.0
    table4_chair2.pose.y = 12.0
    table4_chair2.pose.theta = 3*math.pi/4
    models.append(table4_chair2)

    table4_chair3 = Model()
    table4_chair3.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table4_chair3.name = "8_table5_chair2_3"
    table4_chair3.ns = "namespace1"
    table4_chair3.pose.x = 9.0
    table4_chair3.pose.y = 14.0
    table4_chair3.pose.theta = math.pi/4
    models.append(table4_chair3)

    table4_chair4 = Model()
    table4_chair4.yaml_path = os.path.join(rospack.get_path("simulator_setup"), "static_obstacles", "8_chair2_polygon_4legs_rectangle.yaml")
    table4_chair4.name = "8_table5_chair2_4"
    table4_chair4.ns = "namespace1"
    table4_chair4.pose.x = 11.0
    table4_chair4.pose.y = 12.0
    table4_chair4.pose.theta = 5*math.pi/4
    models.append(table4_chair4)

def respawn_shelves_test():
    # setup respawn_interactive_obstacles
    respawn_interactive_obstacles_service_name = "pedsim_simulator/respawn_interactive_obstacles"
    rospy.wait_for_service(respawn_interactive_obstacles_service_name, 6.0)
    respawn_interactive_obstacles_client = rospy.ServiceProxy(respawn_interactive_obstacles_service_name, SpawnInteractiveObstacles)

    # setup remove_all_interactive_obstacles
    remove_all_interactive_obstacles_service_name = "pedsim_simulator/remove_all_interactive_obstacles"
    rospy.wait_for_service(remove_all_interactive_obstacles_service_name, 6.0)
    remove_all_interactive_obstacles_client = rospy.ServiceProxy(remove_all_interactive_obstacles_service_name, Trigger)

    for i in range(10):
        num = np.random.randint(low=1, high=10)
        msgs = [get_random_shelf() for j in range(num)]
        remove_all_interactive_obstacles_client()
        respawn_interactive_obstacles_client(msgs)
        time.sleep(10)

def example2():
    rospack = rospkg.RosPack()
    respawn_peds_service_name = "pedsim_simulator/respawn_peds"
    rospy.wait_for_service(respawn_peds_service_name, 6.0)
    time.sleep(4)
    respawn_ped_srv = rospy.ServiceProxy(respawn_peds_service_name, SpawnPeds)

    for _ in range(2):
        peds = []

        ped1 = Ped()
        ped1.id = 2
        ped1.pos = Point(3, 10, 0)
        ped1.type = "adult"
        ped1.number_of_peds = 2
        ped1.max_talking_distance = 2.0
        ped1.chatting_probability = 0.0
        ped1.group_talking_probability = 0.0
        ped1.group_talking_base_time = 20.0
        ped1.tell_story_probability = 0.0
        ped1.tell_story_base_time = 20.0
        ped1.talking_and_walking_probability = 0.0
        ped1.talking_and_walking_base_time = 20.0
        ped1.vmax = 2.0
        ped1.force_factor_desired = 1.0
        ped1.force_factor_obstacle = 1.0
        ped1.force_factor_social = 10.0
        ped1.waypoint_mode = 0
        ped1.waypoints = [Point(10, 3, 0.1), Point(7, 15, 0.1)]
        ped1.yaml_file = os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles", "person_two_legged.model.yaml")
        peds.append(ped1)

        # ped2 = copy.copy(ped1)
        # # ped2 = ped1
        # ped2.pos = Point(6, 2, 0.1)
        # ped2.waypoints = [Point(6, 6, 0.1)]
        # peds.append(ped2)


        response = respawn_ped_srv.call(peds)
        print("successfully spawned peds" if response.success else "failed")


        time.sleep(5)


        # peds = []

        # ped2 = Ped()
        # ped2.id = 2
        # ped2.pos = Point(1, 3, 0)
        # ped2.type = 0
        # ped2.number_of_peds = 8
        # ped2.vmax = 1.0
        # ped2.force_factor_desired = 1.0
        # ped2.force_factor_obstacle = 1.0
        # ped2.force_factor_social = 1.0
        # ped2.waypoint_mode = 0
        # ped2.waypoints = [Point(3, 3, 0.1), Point(3, 8, 0.1), Point(8, 8, 0.1), Point(8, 3, 0.1)]
        # ped2.yaml_file = os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles", "person_two_legged.model.yaml")
        # peds.append(ped2)

        # response = respawn_ped_srv.call(peds)
        # print("successfully spawned peds" if response.success else "failed")


        # time.sleep(1)

def service_robot_test():
    rospack = rospkg.RosPack()
    respawn_peds_service_name = "pedsim_simulator/respawn_peds"
    rospy.wait_for_service(respawn_peds_service_name, 6.0)
    respawn_ped_srv = rospy.ServiceProxy(respawn_peds_service_name, SpawnPeds)

    time.sleep(1)

    peds = []

    ped1 = Ped()
    ped1.id = 2
    ped1.pos = Point(8, 3, 0)
    ped1.type = "adult"
    ped1.number_of_peds = 1
    ped1.max_talking_distance = 2.0
    ped1.chatting_probability = 0.0
    ped1.group_talking_probability = 0.0
    ped1.group_talking_base_time = 20.0
    ped1.tell_story_probability = 0.0
    ped1.tell_story_base_time = 20.0
    ped1.talking_and_walking_probability = 0.0
    ped1.talking_and_walking_base_time = 20.0

    ped1.requesting_service_base_time = 30.0
    ped1.receiving_service_base_time = 10.0
    ped1.requesting_service_probability = 0.99

    ped1.vmax = 2.0
    ped1.force_factor_desired = 1.0
    ped1.force_factor_obstacle = 1.0
    ped1.force_factor_social = 1.0
    ped1.waypoint_mode = 0
    ped1.waypoints = [Point(10, 3, 0.1), Point(10, 10, 0.1)]
    ped1.yaml_file = os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles", "person_two_legged.model.yaml")
    peds.append(ped1)


    ped2 = copy.deepcopy(ped1)
    ped2.pos = Point(6, 10, 0.1)
    ped2.type = "servicerobot"
    ped2.number_of_peds = 1
    ped2.max_servicing_radius = 10.0
    ped2.waypoints = [Point(6, 10, 0.1), Point(6, 3, 0.1)]
    ped2.yaml_file = os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles", "servicerobot.model.yaml")
    peds.append(ped2)

    # ped = copy.deepcopy(ped1)
    # ped.pos = Point(1, 10, 0.1)
    # ped.type = "servicerobot"
    # ped.number_of_peds = 1
    # ped.max_servicing_radius = 10.0
    # ped.waypoints = [Point(6, 10, 0.1), Point(6, 3, 0.1)]
    # ped.yaml_file = os.path.join(rospack.get_path("simulator_setup"), "dynamic_obstacles", "servicerobot.model.yaml")
    # peds.append(ped)

    response = respawn_ped_srv.call(peds)



def follow_robot_test():
    respawn_peds_service_name = "pedsim_simulator/respawn_peds"
    rospy.wait_for_service(respawn_peds_service_name, 6.0)
    respawn_ped_srv = rospy.ServiceProxy(respawn_peds_service_name, SpawnPeds)

    time.sleep(1)

    peds = []
    ped = get_default_ped(
            id = 1,
            ped_type = "adult",
            yaml_path = get_yaml_path_from_type("adult"),
            pos = Point(5, 5, 0.1),
            waypoints = [Point(10, 10, 0.1), Point(10, 3, 0.1), Point(3, 10, 0.1), Point(3, 3, 0.1)]
        )
    ped.requesting_service_probability = 0.0
    ped.requesting_guide_probability = 0.05
    peds.append(ped)
    response = respawn_ped_srv.call(peds)
    print("successfully spawned peds" if response.success else "failed")

def get_circle(center, radius):
    obstacles = []
    num_points = 12
    delta_angle = 2 * np.pi / float(num_points)
    rotation = np.array([[np.cos(delta_angle), -np.sin(delta_angle)],
                         [np.sin(delta_angle), np.cos(delta_angle)]])
    points = []
    for i in range(num_points):
        base_vector = np.array([radius, 0])
        # rotate i times
        for _ in range(i):
            base_vector = rotation.dot(base_vector)
        points.append(center + base_vector)

    for i in range(num_points):
        obstacle = LineObstacle()
        start = points[i]
        end = points[(i+1) % num_points]
        obstacle.start = Point(start[0], start[1], 0)
        obstacle.end = Point(end[0], end[1], 0)
        obstacles.append(obstacle)

    return obstacles

def obstacle_force_test():
    # spawn ped service
    spawn_peds_service_name = "pedsim_simulator/spawn_peds"
    rospy.wait_for_service(spawn_peds_service_name, 6.0)
    spawn_peds_client = rospy.ServiceProxy(spawn_peds_service_name, SpawnPeds)

    # add wall service
    add_obstacles_service_name = "pedsim_simulator/add_obstacle"
    rospy.wait_for_service(add_obstacles_service_name, 6.0)
    add_obstacles_client = rospy.ServiceProxy(add_obstacles_service_name, SpawnObstacle)

    ## normal wall
    ped = get_default_ped(
            id = 1,
            ped_type = "adult",
            yaml_path = get_yaml_path_from_type("adult"),
            pos = Point(4, 2, 0.1),
            waypoints = [Point(4, 2, 0.1), Point(12, 2, 0.1)]
        )
    ped.requesting_guide_probability = 0
    ped.requesting_follower_probability = 0
    ped.force_factor_obstacle = 2

    spawn_peds_client.call([ped])

    obstacles = LineObstacles()
    obstacle = LineObstacle()
    obstacle.start = Point(9, 4, 0)
    obstacle.end = Point(9, 1, 0)
    obstacles.obstacles.append(obstacle)

    add_obstacles_client.call(obstacles)

    ## circular wall
    ped = get_default_ped(
            id = 1,
            ped_type = "adult",
            yaml_path = get_yaml_path_from_type("adult"),
            pos = Point(4, 10, 0.1),
            waypoints = [Point(4, 10.2, 0.1), Point(12, 10.2, 0.1)]
        )
    ped.requesting_guide_probability = 0
    ped.requesting_follower_probability = 0
    ped.force_factor_obstacle = 2

    spawn_peds_client.call([ped])

    obstacles = LineObstacles()
    obstacle = LineObstacle()
    obstacle.start = Point(9, 12, 0)
    obstacle.end = Point(9, 8, 0)
    obstacles.obstacles.append(obstacle)
    obstacles.obstacles += get_circle(center=np.array([9, 10]), radius=2.0)

    add_obstacles_client.call(obstacles)

def follow_agent_test():
    # spawn ped service
    spawn_peds_service_name = "pedsim_simulator/spawn_peds"
    rospy.wait_for_service(spawn_peds_service_name, 6.0)
    spawn_peds_client = rospy.ServiceProxy(spawn_peds_service_name, SpawnPeds)

    ped = get_default_ped(
            id = 1,
            ped_type = "adult",
            yaml_path = get_yaml_path_from_type("adult"),
            pos = Point(2, 2, 0.1),
            waypoints = [Point(4, 2, 0.1), Point(12, 2, 0.1)]
        )
    ped.number_of_peds = 3
    ped.requesting_guide_probability = 0
    ped.requesting_service_probability = 0
    ped.requesting_follower_probability = 0.2
    ped.force_factor_robot = 10.0

    spawn_peds_client.call([ped])

if __name__ == '__main__':
    #shelves_test()
    tables_test()
    # service_robot_test()
    # social_force_test()
    # crowd_test()
    # follow_robot_test()
    # example2()
    # respawn_shelves_test()
    # obstacle_force_test()
    # follow_agent_test()
