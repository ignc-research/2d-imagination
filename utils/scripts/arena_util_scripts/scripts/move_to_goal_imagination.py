#!/usr/bin/env python
import rospy
import json
import os
import time
import numpy as np
import cv2 # does not work in (rosnav)
import yaml
import rospkg
import actionlib
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Point, PoseStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction

position_global = Point()

def movebase_client(x, y, goal_pub):
    goal = MoveBaseGoal()
    goal.target_pose.header.seq = 0
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = 1.0

    # version 1: moves the robot & at the goal there is an arrow visualized
    pose_stamped = PoseStamped()
    pose_stamped = goal.target_pose
    goal_pub.publish(pose_stamped)

    # version 2: moves the robot, but no arrow at the goal position
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    client.send_goal(goal)

def callback_odom(data):
    global position_global
    position_global = data.pose.pose.position

# read scenario config file
def main():
    rospack = rospkg.RosPack()
    json_file = rospy.get_param('~json_file') # "scenario1.json" / "scenario8_eval.json" / "map_center.json" / rospy.get_param('~json_file')
    scenario_path = os.path.join(rospack.get_path("simulator_setup"), "training", json_file)
    stream = open(scenario_path, 'r')
    config = json.load(stream)
    print('json file content:\n' + str(config) + '\n')

    num_images = config['num_images'] # no. of images per second # default=20 (or even 40)
    img_sec = int(1.0/(num_images/60.0)) # save an image every X seconds # example: 1/(20/60)=3
    sleep_sec = 0.1 # [sec]
    counter_reset = img_sec/sleep_sec # 15/30/.. # example: 3/0.1=30 # TODO: make img_sec count
    radius = 0.15 # TODO

    goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
    
    for path in config['robot_paths']:
        initial_pos = path['initial_pos'] # TODO: part from the json file, but not used here
        # movebase_client(initial_pos[0], initial_pos[1], goal_pub)
        for subgoal in path['subgoals']:
            rospy.Subscriber("/odom", Odometry, callback_odom)
            robot_pos = np.array([position_global.x, position_global.y])
            goal_distance = np.linalg.norm(robot_pos-subgoal)
            movebase_client(subgoal[0], subgoal[1], goal_pub)
            counter = 0
            while goal_distance > radius*(1 + counter/100): # TODO
                counter +=1
                robot_pos = np.array([position_global.x, position_global.y])
                goal_distance = np.linalg.norm(robot_pos-subgoal)
                rospy.Subscriber("/odom", Odometry, callback_odom)
                if counter % counter_reset == 0:
                    print('set the goal again')
                    movebase_client(subgoal[0], subgoal[1], goal_pub)
                time.sleep(sleep_sec)

def callback_map(map_data):
    map_data_array = np.asarray([map_data.data])

    free_amount = np.sum(map_data_array[0] == 0)
    unknown_amount = np.sum(map_data_array[0] == -1)
    ocupied_amount = np.sum(map_data_array[0] == 100)

    rospack = rospkg.RosPack()
    map_data_array2 = np.array(map_data.data) # array of size 346986

    # get the size of the used map image: width x height = 666 x 521
    map_file = rospy.get_param('~map_file') # "map_empty"
    map_path = rospy.get_param('~map_path')
    with open(map_path, 'r') as stream:
        try:
            doc = yaml.safe_load(stream)
            image = doc['image'] # map_small.png
            map_img_path = os.path.join(rospack.get_path("simulator_setup"), "maps", map_file, image)
        except yaml.YAMLError as exc:
            print(exc)

    used_map_image = cv2.imread(map_img_path)
    map_shape = (used_map_image.shape[0],used_map_image.shape[1])
    map_reshaped = map_data_array2.reshape(map_shape)
    temp = np.ones(map_reshaped.shape)*255
    temp[map_reshaped != -1] = map_reshaped[map_reshaped != -1]

    cv2.imwrite("map_topic.png", temp) # will be saved in folder $HOME\.ros

if __name__ == '__main__':
    rospy.init_node('reach_goal_imagination', anonymous=True)
    rospy.Subscriber('/map', OccupancyGrid, callback_map)
    main()