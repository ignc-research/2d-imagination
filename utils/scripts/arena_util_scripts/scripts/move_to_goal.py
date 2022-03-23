#!/usr/bin/env python
from ctypes import Array
from array import ArrayType
import rospy, rosparam
from math import atan2
import yaml, json
from nav_msgs.msg import Odometry, OccupancyGrid
from rospy.topics import Publisher
from tf.transformations import euler_from_quaternion # to make it work with python3 do: cd $HOME/geometry2_ws $ source devel/setup.bash $ cd $HOME/catkin_ws_ma $ source devel/setup.bash
from geometry_msgs.msg import Point, Twist, PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
import sys
#sys.path.insert(0,'/usr/local/lib/python3.6/dist-packages/cv2/python-3.6/cv2.cpython-36m-x86_64-linux-gnu.so')
#sys.path.insert(0,'/usr/local/lib/python2.7/dist-packages/cv2/python-2.7/cv2.so')
import cv2 # does not work in (rosnav)
import os
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os, glob
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from numpy import asarray
from arena_plan_msgs.msg import IntList, ListIntList, ListListIntList, ListOccupancyGrid
from std_msgs.msg import String
import time

# Important: adjust the local paths! (or copy the 'models' folder into arena-rosnav) (TODO)
# For this to work should both github repositories (arena-rosnav & rosnav-imagination) be placed under src in a catkin workspace!
rospack = rospkg.RosPack()
imagination_path = os.path.join(rospack.get_path("arena_bringup"), "../../rosnav-imagination") # where the repository "https://github.com/ignc-research/rosnav-imagination" has been cloned
sys.path.insert(0,imagination_path)
from rl.semantic_anticipator import SemAnt2D # the model

import torch # works in (rosnav), but there cv2 does not work, as well as tf
from torch.utils.data import Dataset
from torch.utils.data import DataLoader
import torch.optim as optim
from torchvision.utils import save_image
import tensorflow as tf
# change cv2.imread() to torch.load() and cv2.imwrite() to torch.save()!?
# there is also np.save and np.load that always works
# TODO SOLUTION: $ workon rosnav $ pip install opencv-contrib-python # with NO sourcing

x_global = 0.0
y_global = 0.0 
theta_global = 0.0

time_start = 0.0
temp_time = 0.0
img_sec = 1 # default

imagination_counter = 0
timer_done = "no"
sync = 1 # easy change 0 vs. 1

start_path = 0
final_path = 0
paths_between_start_and_end = 1 # easy change 0 vs. 1

move_base_goal = MoveBaseGoal()
goal_global = PoseStamped()
position_global = Point()
position_global.x = 0.0
position_global.y = 0.0
position_global.z = 0.0
orientation_global = Quaternion()

sub_goal = []

global_speed = Twist()

current_model = []
anticipator = []

#map_resolution = 0.0
#x_offset = 0
#y_offset = 0
#x_max = 0
#y_max = 0

def newOdom(msg):
    global x_global
    global y_global
    global theta_global
    x_global = msg.pose.pose.position.x
    y_global = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta_global) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def goal_publisher():
    rospy.init_node("reach_goal", anonymous=True)
    sub = rospy.Subscriber("/odom", Odometry, newOdom) # /odom, /odometry/ground_truth are on the topics list # "/odometry/filtered" does not exist
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    speed = Twist()
    rate = rospy.Rate(10)
    goal = Point()
    goal.x = 13.8938770294
    goal.y = 8.7637386322
    while not rospy.is_shutdown():
        inc_x = goal.x -x_global
        inc_y = goal.y -y_global
        angle_to_goal = atan2(inc_y, inc_x)
        if abs(angle_to_goal - theta_global) > 0.1:
            speed.linear.x = 0.0
            speed.angular.z = 0.3
        else:
            speed.linear.x = 0.5
            speed.angular.z = 0.0
        pub.publish(speed)
        rate.sleep()

def goal_publisher_move_base():
    rospy.init_node("reach_goal", anonymous=True)
    pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        goal = MoveBaseActionGoal()
        goal.header.seq = 0
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = ""
        goal.goal_id.stamp.secs = 0
        goal.goal_id.stamp.nsecs = 0
        goal.goal_id.id = ""
        goal.goal.target_pose.header.seq = 0
        goal.goal.target_pose.header.stamp = rospy.Time.now()
        goal.goal.target_pose.header.frame_id = "map"
        goal.goal.target_pose.pose.position.x = 13.8938770294
        goal.goal.target_pose.pose.position.y = 8.7637386322
        goal.goal.target_pose.pose.position.z = 0.0
        goal.goal.target_pose.pose.orientation.x = 0.0
        goal.goal.target_pose.pose.orientation.y = 0.0
        goal.goal.target_pose.pose.orientation.z = 0.0
        goal.goal.target_pose.pose.orientation.w = 1.0
        pub.publish(goal)
        rate.sleep()

def goal_status_sub_callback(msg): # check if the goal has been reached or not
    for el in msg.status_list:
        if el.status == 3:
            print("GOAL REACHED!") # TODO: should be printed once per reached goal
            break

def callback_move(data): # TODO NEXT: it is not working with a callback
    if sync == 0: print('vel') # delete
    #synchronization(0, 0)
    #time.sleep(5)
    #data_counter = rospy.wait_for_message("/costmap_timer_done", String, timeout=5000)
    global global_speed
    global_speed = data
    
def synchronization(start_path_arg, final_path_arg):
    global sync, start_path, final_path
    if sync == 1: # easy to turn off/on
        # idea1 (implemented): wait for "yes" to be published
        # the robot waits by reaching every goal until the laser scan topic has nothing else to publish for the current robot's position (/move_base/local_costmap/costmap)
        # threading.Timer(5,timeout) has been used
        data_counter = None
        while data_counter is None:
            # Important: timeout [sec]: 5 -> 5000, so give more time !!! it solves the problem!!
            data_counter = rospy.wait_for_message("/costmap_timer_done", String, timeout=5000)
            #data_counter = rospy.wait_for_message("/cmd_vel", Twist, timeout=5000)
            #print(data_counter)
            if paths_between_start_and_end == 1: # a condition to decide if you want all the data, or just the one around the obstacles (without the paths before 'start' and after 'goal')
                if start_path_arg == 1: start_path = 1
                if final_path_arg == 1: final_path = 1
                if start_path_arg == 0: start_path = 0
                if final_path_arg == 0: final_path = 0

        # INFO: the amount of saved images in the folder is 2*imagination_counter
        #print(imagination_counter)

        # idea2: if both "yes" and "no" are published, then wait while "yes" and proceed by "no"
        #print(timer_done)
        #while timer_done == "yes": time.sleep(1)

        # idea3: wait until the pair with the temp_time exists; we take an image every 3 seconds, so check around
        # Problem: the condition underneath is somehow always true, so not appropriate!?
        #path_name_costmap = 'training/' + str(rospy.get_rostime().secs) + "_costmap_part.png"
        #path_name_costmap = 'training2/' + str(temp_time) + "_costmap_part.png"
        path_name_costmap = 'training/' + str(temp_time) + "_costmap_part.png"
        path_name_costmap2 = 'training/' + str(temp_time - 1) + "_costmap_part.png"
        path_name_costmap3 = 'training/' + str(temp_time - 2) + "_costmap_part.png"
        #print(str(path_name_costmap) + " " + str(path_name_costmap2) + " " + str(path_name_costmap3))
        #print(str(rospy.get_rostime().secs) + " " + str(temp_time))
        #if (not os.path.isfile(path_name_costmap)) and (not os.path.isfile(path_name_costmap2)) and (not os.path.isfile(path_name_costmap3)) and (imagination_counter != 0):
        #while (not os.path.isfile(path_name_costmap)) and (not os.path.isfile(path_name_costmap2)) and (not os.path.isfile(path_name_costmap3)) and (imagination_counter != 0):
            # idea3.1: wait for a topic to publish
            #data_counter = rospy.wait_for_message("/imagination_counter", String, timeout=5)
            #data_counter = rospy.wait_for_message("/pair_temp_100x100", ListOccupancyGrid, timeout=5)
            #data_counter = rospy.wait_for_message("/move_base/local_costmap/costmap", OccupancyGrid, timeout=5)
            #data_counter = rospy.wait_for_message("/costmap_timer_done", String, timeout=5)
            # idea3.2: just wait for one of the three images to be created and saved in the folder
            #time.sleep(1) # so the robot waits, but other calculations etc. are still happening

def movebase_client(x, y, goal_pub, start_path_arg, final_path_arg):
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
    #print('C: ' + str(x) + ' ' + str(y)) # debugging

    # INFO: ros time = simulation time & clock time = real time; we work with ros time!

    # TODO NEXT:
    # - wait by every robot movement -> /cmd_vel !? (how oft is it publishing?)
    # -> data = rospy.wait_for_message("/cmd_vel", Twist, timeout=5000)
    # -> callback_move(); /cmd_vel or /odom?
    # -> in a new node? every x seconds synchronize with the costmap!?
    # --> while waiting, the robot does not have to move (vel = 0 or goal = current pos)
    # - try to make the calculations etc. faster!
    # -> maybe the printing is slowing everything down?!
    # -> saving so many things as png. etc. is slowing it down?!
    # -> make a condition to choose if to save only the necessary or everything
    # -> make imagination() more compact, keep only the necessary things
    # -> maybe just rviz visualize it faster!?
    # - calculate and print the delay
    
    # synchronization of the robot's position and the saved laser scan image data
    synchronization(start_path_arg, final_path_arg)

    # TODO: why both publish() and actionlib? (visualize the arrow vs. moves the robot?)
    pose_stamped = PoseStamped()
    pose_stamped = goal.target_pose
    goal_pub.publish(pose_stamped)

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    client.send_goal(goal)
    wait = client.wait_for_result()
    if wait:
        # Important: at this point the goal is just being reached (once per goal), but it is the moment right before the waiting for the rest of the laser scan data!
        # because the waiting is after reaching the goal and before starting moving to the next one
        #print('D: ' + str(x) + ' ' + str(y)) # debugging
        global sub_goal
        sub_goal = [x,y]
        return client.get_result()
    #else:
    #    rospy.logerr("Action server not available!")
    #    rospy.signal_shutdown("Action server not available!")

def goal_publisher_move_base_client():
    goal_status_sub = rospy.Subscriber("/move_base/status", GoalStatusArray, goal_status_sub_callback)
    goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10) # shows the goals (their position and orientation with an arrow)
    # TODO: control the speed, overwrite only the linear.x value of the /cmd_vel topic!?

    # move directly to an example position (for debugging purposes)
    #movebase_client(5, 0, goal_pub, 0, 0) # move just slightly
    #movebase_client(13.8938770294, 4.7637386322, goal_pub, 0, 0) # go in between the tables

    # move zig-zag from left to right
    # the robot automatically waits until the current goal has been reached before pursuing the next one
    x__min = -3
    x__max = 25
    y__min = 0
    y__max = 17.5
    y__step = 2.5
    y__temp = y__min
    while y__temp <= y__max - y__step:
        movebase_client(x__max, y__temp, goal_pub, 0, 0)
        y__temp += y__step
        movebase_client(x__min, y__temp, goal_pub, 0, 0)

    # TODO DEBUGGING gmapping (for scenario 1):
    #movebase_client(5, -1, goal_pub, 0, 0)
    #movebase_client(24, -1, goal_pub, 0, 0)
    #movebase_client(24, 17, goal_pub, 0, 0)
    #movebase_client(5, 17, goal_pub, 0, 0)
    #movebase_client(5, 0, goal_pub, 0, 0)

    # TODO DEBUGGING testing robot's velocity:
    #movebase_client(5, 0, goal_pub, 0, 0)
    #movebase_client(5, 5, goal_pub, 0, 0)
    #movebase_client(0, 5, goal_pub, 0, 0)
    #movebase_client(0, 0, goal_pub, 0, 0)
    #movebase_client(5, 0, goal_pub, 0, 0)

def imagination_global_init():
    #global map_resolution, x_offset, y_offset, x_max, y_max
    #map_resolution = 0.05
    #x_offset = -6
    #y_offset = -6
    #x_max = 521
    #y_max = 666
    map_resolution, x_offset, y_offset, x_max, y_max = get_map_parameters()

    # TODO: at the beginning /imagination_global != /map (init /imagination_global with /map topic)
    white_ar = [255,255,255]
    black_ar = [0,0,0]
    temp_img_grey = np.zeros((x_max,y_max)) # (521,666)
    #temp_img_grey = cv2.imread("map_topic.png")
    #temp_img_grey = cv2.imread("map_global_costmap.png")
    cv2.imwrite("imagination_map_global.png", temp_img_grey)
    cv2.imwrite("imagination_map_global_grey.png", temp_img_grey)
    cv2.imwrite("imagination_map_global_one_color.png", temp_img_grey)
    # publish the imagination costmap to a topic and visualize in rviz (include the topics also directly in the .rviz file)
    # (as an alternative in rviz for Map the default topic /map could be changed to /imagination_global)
    pub2 = rospy.Publisher("/imagination_global", OccupancyGrid, queue_size=10) # (521, 666)
    grid = OccupancyGrid()
    grid.header.seq = 0
    grid.header.stamp = rospy.Time.now()
    grid.header.frame_id = ""
    # get info directly from the subscribed topic, but change width, hight and position!
    grid.info.width = temp_img_grey.shape[1] # 666 !
    grid.info.height = temp_img_grey.shape[0] # 521 !
    grid.info.resolution = map_resolution # 0.05
    #grid.info.map_load_time = map_data.info.map_load_time
    grid.info.origin.orientation.x = 0
    grid.info.origin.orientation.y = 0
    grid.info.origin.orientation.z = 0
    grid.info.origin.orientation.w = 1
    grid.info.origin.position.x = x_offset # ! # -6
    grid.info.origin.position.y = y_offset # ! # -6
    grid.info.origin.position.z = 0
    map_array = []
    i = temp_img_grey.shape[0] - 1
    while i >=0:
        for j in range(temp_img_grey.shape[1]):
            if len(temp_img_grey.shape) < 3: map_array.append(0)
            else:
                #map_array.append(temp_img_grey[i,j])
                BGR_color = [temp_img_grey[i, j, 0], temp_img_grey[i, j, 1], temp_img_grey[i, j, 2]]
                if(BGR_color==black_ar): # black
                    map_array.append(0) # 0 = black = free
                elif(BGR_color==white_ar): # white
                    map_array.append(-1) # -1 = white = unknown
                else: # grey
                    map_array.append(100) # 100 = grey = occupied
        i -= 1
    grid.data = map_array
    pub2.publish(grid)

def get_map_parameters():
    rospack = rospkg.RosPack()
    # get the parameters map_resolution, x_max, y_max, x_offset, y_offset directly from the chosen map
    map_file = rospy.get_param('~map_file') # "map_empty"
    map_path = rospy.get_param('~map_path')
    # parse the .yaml file
    with open(map_path, 'r') as stream:
        try:
            doc = yaml.safe_load(stream)
            map_resolution = float(doc['resolution']) # resolution: 0.05
            x_offset = float(doc['origin'][0]) # origin: [-6.0, -6.0, 0.0]
            y_offset = float(doc['origin'][1])
            image = doc['image'] # image: map_small.png
            # read the image and get its dimensions
            map_img_path = os.path.join(rospack.get_path("simulator_setup"), "maps", map_file, image)
            map_img = cv2.imread(map_img_path)
            x_max = map_img.shape[0] # 521
            y_max = map_img.shape[1] # 666
        except yaml.YAMLError as exc:
            print(exc)
    return map_resolution, x_offset, y_offset, x_max, y_max

def training_script():
    #global map_resolution, x_offset, y_offset, x_max, y_max
    map_resolution, x_offset, y_offset, x_max, y_max = get_map_parameters()
    
    goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10) # shows the goals (their position and orientation with an arrow)
    
    # TODO: at the beginning /imagination_global != /map (init /imagination_global with /map topic)
    imagination_global_init()

    print('\n\n*** Training started ***\n')
    # TODO:
    # start gui with $ python PathCreator.py / $ python PathCreator2.py
    # parse the yaml file created from the gui (arena-tool)
    # - get the name of the map (or names -> then dynamically publish a different map) -> find the png/pgm/yaml map files and load to rviz
    # -/+> get also map resolution and origin?! or just from the scenario name, take the info from scenario_name.yaml!
    # + get the robot's position -> if not the default one, move the robot there first (before starting recording the data)
    # + get the robot's path (waypoints / robot's subgoals) -> move the robot with move_base to each point
    # +> all positions probably in px!? -> convert them! -> all already fine!
    # - get the amount of images per second -> save the laser scan data (local costmap -> image pairs (local costmap part & ground truth part)) in those intervals
    # output: 1000-2000 images => input: 5 scenarios & 3-4 paths per scenario => ca 50-100 'images' per path
    # -/+> the robot should move in each direction with the same velocity (leave it small and just make more 'images')
    
    # TODO: json and not yaml as output!?
    rospack = rospkg.RosPack()
    yaml_file = os.path.join(rospack.get_path("simulator_setup"), "training", "scenario1_idea2.yaml")
    with open(yaml_file, 'r') as stream:
    #with open("$(find simulator_setup)/training/scenario1_idea2.yaml", 'r') as stream:
        try:
            doc = yaml.safe_load(stream)
            print('yaml file content:\n' + str(doc) + '\n')
            # example: {'bodies': [{'footprints': [{'color': 'test', 'type': 'polygon'}, {'color2': 'test2', 'type2': 'polygon2'}], 'name': 'chair_seat'}]}
            # example: {'scenarios': [{'origin': [-6.0, -6.0], 'resolution': 0.05, 'name': 'scenario1', 'ids': [{'robot_path': [[0.0, 0.0], [5.0, 0.0], [10.0, 0.0]], 'img_sec': 2, 'robot_pos': [0.0, 0.0], 'id': 1}, {'id': 2}, {'id': 3}, {'id': 4}]}, {'name': 'scenario2'}, {'name': 'scenario3'}, {'name': 'scenario4'}, {'name': 'scenario5'}], 'scenario': {'name': 'scenario1', 'ids': [{'id': 1}, {'id': 2}, {'id': 3}, {'id': 4}]}}
            
            #for scenario in doc['scenarios']: # idea 1 - one yaml file
            scenario = doc['scenario'] # idea 2 - yaml file per scenario (so 5)
            name = scenario['name']
            res = scenario['resolution']
            origin = scenario['origin']
            for identifier in scenario['ids']:
                id = identifier['id']
                robot_pos = identifier['robot_pos']
                robot_path = identifier['robot_path']
                img_seconds = identifier['img_sec']
            
        except yaml.YAMLError as exc:
            print(exc)

    # One json file per scenario map!
    # TODO: change the json file to a get a different path # TODO X: scenario1_eval.json
    # TODO X: the planer should be better tuned, it still planes to go trought a really small path etc.; the robot should drive slower etc.
    json_file = rospy.get_param('~json_file') # "scenario1.json" / "scenario8_eval.json" / "map_center.json" / rospy.get_param('~json_file')
    json_file_path = os.path.join(rospack.get_path("simulator_setup"), "training", json_file)
    with open(json_file_path, 'r') as stream:
        doc = json.load(stream)
        print('json file content:\n' + str(doc) + '\n')
        # example: {u'num_images': 101, ...}
        
        # Important: in the gui the x-y origin is in the top left corner, but for us it is in the bottom left corner, but
        # all points in the json file already are correctly scaled and transformed, so they are in meters, not in pixels and ready to be used with move_base without any change!
        # that is why we also do not need the parameters 'resolution' and 'origin'
        map = doc['map_path'] # TODO: with or without a path? (in the gui the yaml file for the map should be selected)
        # TODO: 'map' will be a relative path to the yaml file, but
        # first we need the json (for example for scenario1: "scenario1/map.yaml" given, but "scenario1/map_scenario.png" wanted!)
        # and second we need an universal path working from every machine
        # => for example: "/arena-rosnav/simulator_setup/maps/scenario6/map.yaml" -> "scenario6/map.json"
        #map_path = os.path.join(rospack.get_path("simulator_setup"), "maps", map)
        #map_path = os.path.join(rospack.get_path("simulator_setup"), "maps", "scenario1/map_scenario.png")

        # run $ roslaunch arena_bringup pedsim_test.launch obstacles_amount:=26 map_file:=scenario1
        # TODO Question: do not load a scenario from pedsim_test.py -> comment it out in the launch file!?
        # -> if so, obstacles_amount should be changed or something for local costmap to work/be corect/ly!?
        
        #rospy.set_param('~obstacles_amount', 44)
        #rospy.set_param('~map_file', 'scenario2')
        #rospy.set_param("~map_path", "$(find simulator_setup)/maps/$(arg map_file)/map.yaml")
        #rospy.set_param("~world_path", "$(find simulator_setup)/maps/$(arg map_file)/map.world.yaml")
        #rosparam.set_param("~world_path", "$(find simulator_setup)/maps/$(arg map_file)/map.world.yaml")

        # TODO: publish the map?? don't quite work, so for now the name of the map in the json file is not used, the user should start the launch with the right map name by him/herself
        # $roslaunch arena_bringup pedsim_test.launch obstacles_amount:=26 map_file:=scenario1 (even without obstacles_amount, since not needed in laser_scan_data.py, only for the ground truth data generation and showing the obstacle ids in rviz which is not needed for the training)
        pub = rospy.Publisher("/map", OccupancyGrid, queue_size=10)
        grid = OccupancyGrid()
        grid.header.seq = 0
        grid.header.stamp = rospy.Time.now()
        grid.header.frame_id = ""
        grid.info.origin.position.x = 0
        grid.info.origin.position.y = 0
        grid.info.origin.position.z = 0
        grid.info.origin.orientation.x = 0
        grid.info.origin.orientation.y = 0
        grid.info.origin.orientation.z = 0
        grid.info.origin.orientation.w = 1
        #map_image = cv2.imread(map)
        rospack = rospkg.RosPack()
        img_path = os.path.join(rospack.get_path("simulator_setup"), "maps", "scenario1", "map_scenario1.png")
        map_image = cv2.imread(img_path)
        map_array = []
        white_ar = [255,255,255]
        grey_ar = [100,100,100]
        black_ar = [0,0,0]
        i = map_image.shape[0] - 1
        while i >=0:
            for j in range(map_image.shape[1]):
                #map_array.append(map_image[i,j])
                BGR_color = [map_image[i, j, 0], map_image[i, j, 1], map_image[i, j, 2]]
                if(BGR_color==black_ar): # black
                    map_array.append(0) # 0 = black = free
                elif(BGR_color==white_ar): # white
                    map_array.append(-1) # -1 = white = unknown
                else: # grey
                    map_array.append(100) # 100 = grey = occupied
            i -= 1
        grid.data = map_array
        #pub.publish(grid) # TODO

        global time_start, temp_time, img_sec
        # TODO: amount of images per path vs. images per second -> images per second is easier since it could be the same for all paths in a scenario even if the paths have a different length
        num_images = doc['num_images'] # number of images per minute so that we always have an integer value; max: 1 img per millisecond, min: 1 img per minute
        img_sec = int(1.0/(num_images/60.0)) # no. of images = 20 => sec = 3 (for debugging)
        #img_sec = 10 # debugging
        for path in doc['robot_paths']: # the paths normally start and end around the obstacles (so that possibly there are no empty images - without parts of obstacles)
            initial_pos = path['initial_pos']
            pub_counter = rospy.Publisher("/imagination_counter", String, queue_size=10)
            pub_counter.publish(str(0)) # initial value
            #print(rospy.get_rostime().secs)
            # rospy.Time.now() vs. rospy.get_time() vs. rospy.get_rostime().secs
            #time_start = rospy.get_time() # Get the current time in float seconds.
            time_start = rospy.get_rostime().secs # get only the seconds (it's enough?), then the number do not have to be converted to integer while comparing the times
            temp_time = rospy.get_rostime().secs
            # get the pair images costmap_part-ground_truth_map_part by subscribing to the new topics from the 'scan_values' node
            #sub_costmap = rospy.Subscriber("costmap_temp", Image, callback_costmap_temp)
            #sub_costmap = rospy.Subscriber("costmap_temp", IntList, callback_costmap_temp)
            #sub_costmap = rospy.Subscriber("costmap_temp", OccupancyGrid, callback_costmap_temp)
            #sub_ground_truth_map = rospy.Subscriber("ground_truth_map_temp", Image, callback_ground_truth_map_temp)
            #sub_ground_truth_map = rospy.Subscriber("ground_truth_map_temp", IntList, callback_ground_truth_map_temp)
            #sub_ground_truth_map = rospy.Subscriber("ground_truth_map_temp", OccupancyGrid, callback_ground_truth_map_temp)

            imagination_size = int(rospy.get_param('~imagination_size')) # 60/80/100 # TODO X
            if imagination_size == 60: rospy.Subscriber("pair_temp_60x60", ListOccupancyGrid, callback_pair_temp_60x60)
            if imagination_size == 80: rospy.Subscriber("pair_temp_80x80", ListOccupancyGrid, callback_pair_temp_80x80)
            if imagination_size == 100: rospy.Subscriber("pair_temp_100x100", ListOccupancyGrid, callback_pair_temp_100x100)
            
            #sub_obstacles_map = rospy.Subscriber("obstacles_map_temp", Image, callback_obstacles_map_temp)
            #sub_obstacles_map = rospy.Subscriber("obstacles_map_temp", IntList, callback_obstacles_map_temp)
            global sub_goal
            sub_goal = initial_pos
            movebase_client(initial_pos[0], initial_pos[1], goal_pub, 1, 0) # first move to the initial pose
            for subgoal in path['subgoals']:
                # the robot automatically waits until the current goal has been reached before pursuing the next one
                #print(rospy.get_rostime().secs)
            #    sub_goal = subgoal # now done in movebase_client()
                #print('A: ' + str(subgoal)) # debugging
                rospy.Subscriber("/odom", Odometry, callback_odom)
                global position_global
                radius = 0.1 # TODO: should be tuned!
                # Important: do-while-loop, publish the same goal, until it has been reached and only after that move to the next one !!
                movebase_client(subgoal[0], subgoal[1], goal_pub, 0, 0)
                ##while (position_global.x != subgoal[0] and position_global.y != subgoal[1]):
                while not ((position_global.x <= (subgoal[0] + radius) and position_global.x >= (subgoal[0] - radius)) and (position_global.y <= (subgoal[1] + radius) and position_global.y >= (subgoal[1] - radius))):
                    #print("must be: " + str(subgoal) + ", is: " + str(position_global.x) + "," + str(position_global.y)) # debugging
                    movebase_client(subgoal[0], subgoal[1], goal_pub, 0, 0)
                # TODO X NEXT: tune the robot's orientation at each subgoal?! or just set a bigger radius around the goal that passes as 'goal reached'
                # -> change parameters xy_goal_tolerance and yaw_goal_tolerance to a bigger values!? in /arena-rosnav/arena_navigation/arena_local_planner/model_based/conventional/config/base_local_planner_params.yaml but no change visible!?
                # -> pass the current orientation of the robot forward (the last orientation towards reaching the goal, in radius 0.1 for example)!? -> orientation_global
            # unsubscribe until the next initial_pos has been reached and then subscribe again:
            # but since computation of loca costmap takes time, we may lose the last data if we unsubscribe, so better don't unsubscribe here and spend more time at the end deleting the black images
            #sub_costmap.unregister()
            #sub_ground_truth_map.unregister()
            #sub_obstacles_map.unregister()
        # TODO: since the laser node computes slowly, the first couple of images in the training folder are black and the couple of images of the path are missing!
        final_path = 1 # TODO
        # going back to the start maybe be not needed anymore after the robot is waiting for the laser scan data at each goal point, so no laser scan data will be missed already at the last goal point
        sub_goal = [0,0]
        movebase_client(0, 0, goal_pub, 0, 1) # maybe at the end move again to origin (0,0) to get a chance to collect the rest of the local_costmap data
        #final_path = 0
        #sub_pair_map_100x100.shutdown() # TODO NEXT: it does not do anything after that, because it waits those 5 min for ever
        # TODO: maybe it helps, if that is ran separately, when the rest before that is completely ready!? - no difference
        # TODO: wait until the robot has reached its final goal and is not moving
        # TODO: maybe delete the images directly after they have been created, or directly do not save black images
        goal_status_sub = rospy.Subscriber("/move_base/status", GoalStatusArray, goal_status_sub_callback)
        # calling the function maybe be almost not needed anymore after the data before the start of a path and after the end of a path is not taken anymore, since a path not near an obstale shouldn't be drawn on the first place, so empty, black images are therefore not expected
        delete_empty_images_get_raw_data()

def type_color_reference():
    id_type_color_ar = [] # TODO: better parse it from somewhere
    id_type_color_ar.append({'id' : 1, 'color' : (0.3,0.0,0.1)})
    id_type_color_ar.append({'id' : 2, 'color' : (0.0,0.6,0.1)})
    id_type_color_ar.append({'id' : 3, 'color' : (0.1,0.6,1.0)})
    id_type_color_ar.append({'id' : 4, 'color' : (0.4,0.0,0.5)})
    id_type_color_ar.append({'id' : 5, 'color' : (0.5,0.1,0.0)})
    id_type_color_ar.append({'id' : 6, 'color' : (0.6,1.0,0.3)})
    id_type_color_ar.append({'id' : 7, 'color' : (0.7,0.0,0.0)})
    id_type_color_ar.append({'id' : 8, 'color' : (0.7,0.0,0.5)})
    id_type_color_ar.append({'id' : 9, 'color' : (0.7,0.3,0.6)})
    id_type_color_ar.append({'id' : 10, 'color' : (0.8,0.5,0.1)})
    return id_type_color_ar

def delete_empty_images_get_raw_data():
    # the next two lines are new, needed because of the robot synchronization (only if turned on!) (=> if you just need to run only this function, comment out the lines)
    goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10) # shows the goals (their position and orientation with an arrow)
    movebase_client(0, 0, goal_pub, 0, 0)

    # 1) open each img in the training folder and if it is complety black delete it, since they are not important/useful for the training
    # TODO NEXT: it could happen that the costmap is empty, but the gt pair is not, so either delete both or do not delete either of them!?!
    # -> a quick check is that the number of images should be an odd number!
    # -> it is also better to run this function separately at the end once it was made sure that the image pairs are fine and final!
    # 2) get the raw data from every image and save it into a npy file
    print('\nPreparing the raw training data ...\n')
    path = './training'
    container_costmap_color = [] # TODO: one for ground truth data and one for costmap data (both arrays should have the same order!)
    container_ground_truth_color = []
    container_costmap_id = []
    container_ground_truth_id = []
    #container_dict = [] # TODO: use a dict instead, so that there is only one npz file (just appending costmap, gt, costmap, gt etc. won't work, since npz do not keep the order!?)
    #dict = {'costmap' : 'asarray(filename1)', 'ground_truth' : 'asarray(filename2)'}
    id_type_color_ar = type_color_reference()

    # TODO NEXT: is it sure that the images are taken alphabetically? at least in pairs? -> sorted() ?!
    img_count = 0
    for filename in sorted(glob.glob(os.path.join(path, '*.png'))):
        img = cv2.imread(filename)
        all_black = True
        black_ar = [0,0,0]
        #print(filename) # for debugging
        # skip the name of a file, which has already been deleted, because its paired, empty costmap or ground truth map has been deleted
        if img is None: continue
        img_count += 1
        img_id = np.zeros((img.shape[0],img.shape[1])) # (row,col) vs. (row,col,3)
        for i in range(img.shape[0]):
            for j in range(img.shape[1]):
                BGR_color = [img[i, j, 0], img[i, j, 1], img[i, j, 2]]
                id_temp = 0 # per default 0 = free = no obstacles
                if BGR_color != black_ar:
                    all_black = False
                    #print('COLORED PIXEL: ' + str(BGR_color)) # for debugging
                # map here the color to the id:
                for elem in id_type_color_ar:
                    #print('RED from RGB: ' + str(int(elem['color'][0]*255)) + ' ' + str(BGR_color[0])) # for debugging
                    # Important: RGB vs. BGR!
                    if 100 == BGR_color[2] and 100 == BGR_color[1] and 100 == BGR_color[0]:
                        id_temp = 100 # TODO: 100 or 1 .. (should be the same as 'GREY DEFAULT')
                    if 255 == BGR_color[2] and 255 == BGR_color[1] and 255 == BGR_color[0]:
                        id_temp = 50 # TODO X: 1/50/-1/..?
                    if int(elem['color'][0]*255) == BGR_color[2] and int(elem['color'][1]*255) == BGR_color[1] and int(elem['color'][2]*255) == BGR_color[0]:
                        id_temp = elem['id']
                        break
                img_id[i,j] = id_temp

        if all_black:
            os.remove(filename) # delete the image
            # TODO: delete also the second half of the pair!
            if ''.join(filename.split('costmap')) != filename: # costmap deleted -> delete ground truth
                filename_paired = 'ground_truth_map'.join(filename.split('costmap'))
                # check if the file exists
                if os.path.isfile(filename_paired):
                    os.remove(filename_paired)
            else: # ground truth deleted -> delete costmap
                filename_paired = 'costmap'.join(filename.split('ground_truth_map'))
                if os.path.isfile(filename_paired):
                    os.remove(filename_paired)
        else:
            img_ar = asarray(img) # img_ar.shape = (60,60) # rgb color
            img_id_ar = asarray(img_id) # img_id_ar.shape = (60,60) # id
            #print(img_ar) # [[ 0  0  0] [ 0  0  0] ..., [ 0  0  0] [ 0  0  0]] # rgb color
            #print(img_id_ar) # [[ 0. 0. ..., 0. 0.] [ 0. 0. ..., 0. 0.] ..., [ 0. 0. ..., 0. 0.]] # id
            
            # TODO X: if the costmap and gt should be smaller (for example if it is 100x100, but should be 60x60 or 80x80)
            img_range = img.shape[0] # 60/80/100
            resize_big_to_small = 0
            new_range = 60 # should be smaller then img_range (80/60)
            step = int((img_range - new_range)/2)
            if (resize_big_to_small == 1) and (img_range > new_range):
                img_ar_new = [img_ar[i][step:img_range-step] for i in range(step,img_range-step)]
                img_ar = asarray(img_ar_new)
                img_id_ar_new = [img_id_ar[i][step:img_range-step] for i in range(step,img_range-step)]
                img_id_ar = asarray(img_id_ar_new)
                # overwrite also the images themselves (both costmap and gt)
                cv2.imwrite(filename, img_ar) # img_ar is enough, img_id_ar is only internal

            if ''.join(filename.split('costmap')) != filename: # costmap
                filename_paired = 'ground_truth_map'.join(filename.split('costmap'))
                # TODO: check if the current image has a pair, if not - delete it!
                if not(os.path.isfile(filename_paired)):
                    os.remove(filename)
                else:
                    container_costmap_color.append(img_ar)
                    container_costmap_id.append(img_id_ar)
            else: # ground truth
                filename_paired = 'costmap'.join(filename.split('ground_truth_map'))
                if not(os.path.isfile(filename_paired)):
                    os.remove(filename)
                else:
                    container_ground_truth_color.append(img_ar)
                    container_ground_truth_id.append(img_id_ar)

            # DEBUGGING: write/overwrite an image and then load it again to check how it is visualized
            #np.save('training/00_test.npy', img_id_ar) # img_ar vs. img_id_ar
            #img_loaded = np.load('training/00_test.npy')
            #print(img_loaded) # the same as print(img_id_ar)
            #plt.imshow(img_loaded) # view the images
            #plt.show() # all correct, only different colors used, but that is because the id array was used!?!

    # color vs. id -> have both options saved!
    np.savez('training/1_1_container_costmap_color.npz', *container_costmap_color)
    np.savez('training/1_2_container_ground_truth_color.npz', *container_ground_truth_color)
    np.savez('training/2_1_container_costmap_id.npz', *container_costmap_id)
    np.savez('training/2_2_container_ground_truth_id.npz', *container_ground_truth_id)

    # DEBUGGING: load an already existing example image
    #img_test = cv2.imread('training/01_test.png')
    #img_test_ar = asarray(img_test)
    #np.save('training/01_test.npy', img_test_ar)
    #img_loaded_temp = np.load('training/01_test.npy')
    #print(img_loaded_temp)
    #plt.imshow(img_loaded_temp) # plt.imshow(img_loaded_temp, cmap='gray')
    #plt.show() # it is showing the same image as 01_test.png only not in RGB, but in BGR format (or the other way around)

    # TODO NEXT:
    # both files should display the costmap / the ground truth of the same image!
#    file_2_1 = np.load('training/2_1_container_costmap_id.npz')
#    file_2_2 = np.load('training/2_2_container_ground_truth_id.npz')
    #plt.imshow(file_2_1['arr_60'])
    #plt.show()
    #plt.imshow(file_2_2['arr_60'])
    #plt.show()
#    for i in range(int(img_count/2)): # starts with arr_0, all together img_count/2 files
#        plt.imshow(file_2_1['arr_' + str(i)])
#        #plt.show()
#        plt.imshow(file_2_2['arr_' + str(i)])
#        #plt.show()

    print('DONE!')

def get_id_from_color(img_costmap_color):
    img_costmap_id = np.zeros((img_costmap_color.shape[0],img_costmap_color.shape[1])) # (row,col) vs. (row,col,3)
    id_type_color_ar = type_color_reference()
    for i in range(img_costmap_color.shape[0]):
        for j in range(img_costmap_color.shape[1]):
            BGR_color = [img_costmap_color[i, j, 0], img_costmap_color[i, j, 1], img_costmap_color[i, j, 2]]
            id_temp = 0 # per default 0 = free = no obstacles
            if 100 == BGR_color[2] and 100 == BGR_color[1] and 100 == BGR_color[0]:
                id_temp = 100 # TODO: 100 or 1 .. (should be the same as 'GREY DEFAULT')
            if 255 == BGR_color[2] and 255 == BGR_color[1] and 255 == BGR_color[0]:
                id_temp = -1 # TODO X: 1/50/-1/..?
            # map here the color to the id:
            for elem in id_type_color_ar:
                #print('RED from RGB: ' + str(int(elem['color'][0]*255)) + ' ' + str(BGR_color[0])) # for debugging
                # Important: RGB vs. BGR!
                if int(elem['color'][0]*255) == BGR_color[2] and int(elem['color'][1]*255) == BGR_color[1] and int(elem['color'][2]*255) == BGR_color[0]:
                    id_temp = elem['id']
                    #id_temp = 100 # 1 or 100 (should be the same as 'GREY DEFAULT') # one layer for now! already done in CustomDatasetSingle() (TODO)
                    break
            img_costmap_id[i,j] = id_temp
    return img_costmap_id

def get_color_from_id(id):
    id_type_color_ar = type_color_reference()
    grey_ar = [100,100,100]
    black_ar = [0,0,0]
    white_ar = [255,255,255]
    color_BGR_temp = black_ar
    if id == 100: color_BGR_temp = grey_ar # 1 or 100 (should be the same as 'GREY DEFAULT')
    if id == -1: color_BGR_temp = white_ar # TODO X
    else:
        for elem in id_type_color_ar:
            if elem['id'] == id:
                # Important: RGB vs. BGR!
                color_BGR_temp = [int(elem['color'][2]*255), int(elem['color'][1]*255), int(elem['color'][0]*255)]
                #color_BGR_temp = grey_ar # 1 or 100 (should be the same as 'GREY DEFAULT') # one layer for now! already done in CustomDatasetSingle() (TODO)
                break
    return color_BGR_temp

def get_color_from_id_array(ar):
    row,col = ar.shape
    img = np.zeros((row,col,3)) # (row,col) vs. (row,col,3); (row,col,3), because a color, for example the black color (0,0,0) is needed!
    for i in range(row):
        for j in range(col):
            img[i,j] = get_color_from_id(ar[i,j])
    return img

class CustomDatasetSingle(Dataset): # (TODO) test dataset generator for a single npy file
    def __init__(self, ground_truth, costmap, num_catagories, catagories):
        self.ground_truth = ground_truth
        self.costmap = costmap
        self.num_catagories = num_catagories
        self.catagories = catagories
    
    def __len__(self):
        return 1
    
    def __getitem__(self,idx):
        gt = np.zeros((self.num_catagories, self.ground_truth.shape[0], self.ground_truth.shape[1]))
        for i in range(self.num_catagories):
            gt[i][self.ground_truth > 0] = 1 # one layer ground truth data for now, so occupied or not
            #gt[i][self.ground_truth.all() > 0 and self.ground_truth.all() != 100] = 1 # one layer ground truth data for now, so occupied or not # TODO
            # -> 1 as just a random, easy id, or 100 if you want to refer to the color grey (should be the same as 'GREY DEFAULT')
            #gt[i][self.ground_truth > 0.2] = 1 # one layer ground truth data for now, so occupied or not
        lidar = self.costmap
        return lidar[np.newaxis], gt

# TODO NEXT: it looks like the data is taken the whole time and not at first when the robot reaches the starting point?!?
# TODO NEXT: put the imagination part in a separate function, so that it could be called for testing but commented out while collecting training data
# get_single_raw_data -> imagination
def imagination(map_data, img_name, costmap_gt_range): # TODO: get the raw data from an image and save it into a npy file
    #global map_resolution, x_offset, y_offset, x_max, y_max
    #x_offset = -6
    #y_offset = -6
    #x_max = 521
    #y_max = 666
    map_resolution, x_offset, y_offset, x_max, y_max = get_map_parameters()
    
    print('\nImagination in process ...\n')
    # costmap_color, ground_truth_color, costmap_id, ground_truth_id
    img_costmap_color = cv2.imread(img_name)
    img_costmap_id = get_id_from_color(img_costmap_color)
    img_costmap_color_ar = np.asarray(img_costmap_color) # img_ar.shape = (60,60) # rgb color
    img_costmap_id_ar = np.asarray(img_costmap_id) # img_id_ar.shape = (60,60) # id
    #print(img_ar) # [[ 0  0  0] [ 0  0  0] ..., [ 0  0  0] [ 0  0  0]]] # rgb color
    #print(img_id_ar) # [[ 0. 0. ..., 0. 0.] [ 0. 0. ..., 0. 0.] ..., [ 0. 0. ..., 0. 0.]] # id
    filename_paired = 'ground_truth_map'.join(img_name.split('costmap'))
    # check if the current image has a pair, if not - do not do anything!
    if os.path.isfile(filename_paired):
        img_ground_truth_color = cv2.imread(filename_paired)
        img_ground_truth_id = get_id_from_color(img_ground_truth_color)
        img_ground_truth_color_ar = np.asarray(img_ground_truth_color)
        img_ground_truth_id_ar = np.asarray(img_ground_truth_id)
    else: return

    # convert img_ground_truth_id_ar and img_costmap_id_ar to npy files, keeping the same timestamp, to load them back as arguments in the function CustomDatasetSingle()
    # it is not needed though, directly the arrays can be arguments in the function CustomDatasetSingle()
    #path_name_costmap_id_npy = '.npy'.join(img_name.split('.png'))
    #path_name_ground_truth_id_npy = '.npy'.join(filename_paired.split('.png'))
    #np.save(path_name_ground_truth_id_npy, img_ground_truth_id_ar)
    #img_ground_truth_id_npy = np.load(path_name_ground_truth_id_npy)
    #np.save(path_name_costmap_id_npy, img_costmap_id_ar)
    #img_costmap_id_npy = np.load(path_name_costmap_id_npy)
    ##plt.imshow(img_costmap_id_npy)
    ##plt.show()
    
    # put the costmap into a model and get the imagination costmap (the observation)
    # for now the id costmap should contain only 0 and 1!
    num_catagories = 1
    catagories = [0]
    batch_size = 8
    num_import_layers = 1
    num_output_layers = num_catagories # for now =1, extend later on!
    network_size = 32 # 16/32/64
    device = rospy.get_param('~device') # 'cpu'/'cuda'/'cuda:0'

    #MapDatasetTestNPY = CustomDatasetSingle(img_ground_truth_id_npy, img_costmap_id_npy, num_catagories, catagories)
    MapDatasetTestNPY = CustomDatasetSingle(img_ground_truth_id_ar, img_costmap_id_ar, num_catagories, catagories)
    test_dataloader_npy = DataLoader(MapDatasetTestNPY, batch_size, shuffle=False)

    lidar_test_npy, labels_test_npy = next(iter(test_dataloader_npy))
    lidar_test_npy = lidar_test_npy.type(torch.float32).to(device)
    labels_test_npy = labels_test_npy.type(torch.float32).to(device)

    # check labels_test_npy[0,0] and lidar_test_npy[0,0]
    #path_name_labels_png = 'labels'.join(img_name.split('costmap'))
    #path_name_lidar_png = 'lidar'.join(img_name.split('costmap'))
    #save_image(labels_test_npy, path_name_labels_png) # good! the same as the gt image only in black and white
    #save_image(lidar_test_npy, path_name_lidar_png) # good! the same as the costmap image only in black and white
    ##print(labels_test_npy) # tensor() array with elements 0. or 1.

    # Important: adjust the local paths! (or copy the 'models' folder into arena-rosnav) (TODO)
    # rospy.get_param() works here and not globally!?
#    node_user = rospy.get_param('~user')
#    node_workspace = rospy.get_param('~workspace')
#    imagination_path = '/home/' + node_user + '/' + node_workspace + '/src/rosnav-imagination' # where the repository "https://github.com/ignc-research/rosnav-imagination" has been cloned
#    sys.path.insert(0,imagination_path)
#    from rl.semantic_anticipator import SemAnt2D # the model

    #global imagination_path

    # TODO X: sort the collected data and the trained models in folders
    # Load the model and get the prediction (the label)
    group_number = 1 # 1 for group "models" & 2 for group "models_state_dict"
    current_model_number = 0
    if imagination_counter == 0: # load the model only once! (TODO X)
        if group_number == 1:
            ## a model from group "models"
            current_model_number = rospy.get_param('~imagination_model') # "3000"/"3000_60_normal"/"3000_100_normal"/rospy.get_param('~imagination_model')...
            # Important: CPU vs. GPU!
            global current_model
            if device == 'cpu': current_model = torch.load(imagination_path + "/example/models/model_" + str(current_model_number) + ".pth", map_location=torch.device(device)) # CPU
            else: current_model = torch.load(imagination_path + "/example/models/model_" + str(current_model_number) + ".pth").to(device) # GPU
            #print(current_model) # SemAnt2D(...)
            current_model.eval()
        else: # now it is used only for testing purposes
            ## a model from group "models_state_dict"
            current_model_number = 2760 # 2760/...
            # Important: CPU vs. GPU!
            if device == 'cpu': current_model_state_dict = torch.load(imagination_path + "/example/models_state_dict/model_" + str(current_model_number) + ".pth", map_location=torch.device(device)) # CPU
            else: current_model = torch.load(imagination_path + "/example/models_state_dict/model_" + str(current_model_number) + ".pth").to(device) # GPU
            #print(current_model_state_dict) # tensor() arrays
            global anticipator
            anticipator = SemAnt2D(num_import_layers,num_output_layers,network_size).to(device) # init the model
            anticipator.load_state_dict(current_model_state_dict['model_state_dict']) # ! otherwiese 'current_model_labels_prediction_npy = current_model_state_dict(lidar_test_npy)' gives an error that 'dict' object is not callable
            #print(anticipator) # SemAnt2D(...)
            anticipator.eval()
            current_model_labels_prediction_npy = anticipator(lidar_test_npy) # anticipator() / current_model()
    if group_number == 1:
        current_model_labels_prediction_npy = current_model(lidar_test_npy)
    else:
        current_model_labels_prediction_npy = anticipator(lidar_test_npy) # anticipator() / current_model()

    # save the imagination costmap (the observation) as png and npy file (cv2.imwrite, torch.save, np.save, save_image (black img as a result), plt.imsave (works!))
    #print(labels_test_npy.shape) # torch.Size([1, 1, 60, 60])
    #print(current_model_labels_prediction_npy["occ_estimate"].detach()) # prediction # tensor() array with float elements # tensor([[[[1.3750e-06, 6.0028e-08, 2.5874e-08,  ..., 3.5876e-05, ...]]]])
    path_name_current_imagination_map_id_png = ('imagination_map_' + str(current_model_number)).join(img_name.split('costmap')) # imagination map = prediction map/costmap
    plt.imsave(path_name_current_imagination_map_id_png, current_model_labels_prediction_npy["occ_estimate"].detach()[0,0]) # it works only with [0,0] at the end!?
    imagination_map = cv2.imread(path_name_current_imagination_map_id_png)
    #path_name_current_imagination_map_id_npy = '.npy'.join(path_name_current_imagination_map_id_png.split('.png')) # imagination map = prediction map/costmap
    #np.save(path_name_current_imagination_map_id_npy, current_model_labels_prediction_npy["occ_estimate"].detach().numpy()) # not needed

    # FILTER 1: try to filter out false imaginated parts from the output
    # normalize the tensor torch array (get values between 0 and 1, to be able to filter some out)
    imagination_filter1_threshold = rospy.get_param('~imagination_filter1_threshold') # 0.1/0.2/0.3 # probability between 0.0 and 1.0 TODO X: tune the number
    #print(imagination_map) # (100,100,3) # color ~ (84 1 68)
    #print(current_model_labels_prediction_npy["occ_estimate"].detach()[0,0].shape) # torch.Size([100, 100]) # tensor([[2.4808e-05, 2.6347e-06, 1.5144e-06,  ..., 3.1066e-05, 6.4751e-05, 2.0734e-04], [...], ...])
    inputs = current_model_labels_prediction_npy["occ_estimate"].detach()[0,0]
    #inputs_normal = inputs/tf.reduce_max(tf.abs(inputs)) # doesn't work # TypeError: Cannot interpret 'tf.float32' as a data type
    inputs_np = inputs.numpy() # [[2.4808e-05, 2.6347e-06, 1.5144e-06,  ..., 3.1066e-05, 6.4751e-05, 2.0734e-04], [...], ...]
    inputs_np /= np.max(inputs_np) # works # [[0.6392533  0.64117384 0.6324189  ... 0.34298816 0.3978336  0.5802038 ], ... ,[1. 1. 1. ... 1. 1. 1.]]
    #path_name_imagination_map_id_png_np = 'imagination_map_np'.join(img_name.split('costmap'))
    #plt.imsave(path_name_imagination_map_id_png_np, inputs_np) # looks exactly like imagination_map, so just like before the transformation from torch to np array
    if imagination_filter1_threshold > 0:
        for i in range(img_costmap_color.shape[0]):
            for j in range(img_costmap_color.shape[1]):
                if inputs_np[i,j] < imagination_filter1_threshold: inputs_np[i,j] = 0 # filter out values < imagination_filter1_threshold (make them = 0)
        # save the filtered image to compare with the original imagination
        path_name_imagination_map_id_png_filtered_value = 'imagination_map_filtered_value'.join(img_name.split('costmap'))
        #plt.imsave(path_name_imagination_map_id_png_filtered_value, inputs_np) # for debugging
        #inputs_torch = torch.from_numpy(inputs_np) # is the same image as inputs_np
        # filter the image (overwrite both the imagination variable and the saved imagination image)
        plt.imsave(path_name_current_imagination_map_id_png, inputs_np)
        imagination_map = cv2.imread(path_name_current_imagination_map_id_png) # TODO: write and read to get the right folrm!?
    
    # FILTER 2: draw a circle around every colored pixel in the laser scan (TODO X)
    # -> check img_costmap_color for a color different then black and white
    # -> save the (x,y) and create a mask (black image at the beginning) with a filled white circle around every point with a certain radius
    # --> at the beginning draw not filled colored circles for debuging
    # -> use the mask on top of the imagination image to filter it
    imagination_filter2_range = rospy.get_param('~imagination_filter2_range') # radius 10 px = 0.5 m
    if imagination_filter2_range > 0:
        indexes_center_filter_circles_ar = []
        for i in range(img_costmap_color.shape[0]):
            for j in range(img_costmap_color.shape[1]):
                if img_costmap_color[i,j][0] == 0 and img_costmap_color[i,j][1] == 0 and img_costmap_color[i,j][2] == 0: # black = free
                    continue
                elif img_costmap_color[i,j][0] == 255 and img_costmap_color[i,j][1] == 255 and img_costmap_color[i,j][2] == 255: # white = the border
                    continue
                else:
                    indexes_center_filter_circles_ar.append([i,j])
        #print(indexes_center_filter_circles_ar) # for debuging
        #print(len(indexes_center_filter_circles_ar)) # for debuging
        image_filter = np.zeros((img_costmap_color.shape[0],img_costmap_color.shape[1],3)) # (row,col) vs. (row,col,3)
        # draw a white filled circle with a center at each collected index in the array
        for index in range(len(indexes_center_filter_circles_ar)):
            cv2.circle(image_filter, (indexes_center_filter_circles_ar[index][1],indexes_center_filter_circles_ar[index][0]), imagination_filter2_range, (255,255,255), -1)
        # draw a red not filled circle with a center at each collected index in the array for debugging
    #    for index in range(len(indexes_center_filter_circles_ar)):
    #        cv2.circle(image_filter, (indexes_center_filter_circles_ar[index][1],indexes_center_filter_circles_ar[index][0]), imagination_filter2_range, (255,0,0), 1)
        # TODO: write & read the ROI image to get the right form etc. !? (needed)
        cv2.imwrite('roi_filter2.png', image_filter)
        image_filter_roi = cv2.imread('roi_filter2.png')
        # create the mask (image_filter is 3-channel, mask must be single channel => cvtColor())
        image_filter_roi2gray = cv2.cvtColor(image_filter_roi,cv2.COLOR_BGR2GRAY)
        ret, mask1 = cv2.threshold(image_filter_roi2gray, 10, 255, cv2.THRESH_BINARY)
    #    path_name_imagination_map_filter2_png = 'imagination_map_filter2'.join(img_name.split('costmap'))
    #    cv2.imwrite(path_name_imagination_map_filter2_png, image_filter) # for debugging
        # filter/mask the image
        imagination_map = cv2.bitwise_and(imagination_map,imagination_map,mask = mask1)
        cv2.imwrite(path_name_current_imagination_map_id_png, imagination_map)
    
    # - FILTER 3 (not used) (TODO X): do imagination only in a circle with radius A around the robot, so the false imaginations on the edges will be filtered out
    # -> the robot is always in the middle of the model input images
    # -> use an opencv mask and the function cv2.bitwise_and()
    imagination_mask_filter_bool = 0
    if imagination_mask_filter_bool == 1:
        x=y=costmap_gt_range # could be 60/80/100/..
        image_filter = np.zeros((x,y,3)) # ROI (region of interest)
        filter_circle_radius_px = int(costmap_gt_range/2)-10 # 10 / 20 / .. / int(costmap_gt_range/2)=max
        # draw a white filled circle in the middle with a variable radius
        cv2.circle(image_filter, (int(x/2),int(y/2)), filter_circle_radius_px, (255,255,255), -1)
        # TODO: write & read the ROI image to get the right form etc. !? (needed)
        cv2.imwrite('roi.png', image_filter)
        image_filter_roi = cv2.imread('roi.png')
        # create the mask (image_filter is 3-channel, mask must be single channel => cvtColor())
        image_filter_roi2gray = cv2.cvtColor(image_filter_roi,cv2.COLOR_BGR2GRAY)
        ret, mask2 = cv2.threshold(image_filter_roi2gray, 10, 255, cv2.THRESH_BINARY)
        #cv2.imwrite('roi_mask.png', mask2) # = 'roi.png'
        # TODO X: mask the image, black-out/filter-out the area outside of the circle with a variable radius around the robot
        imagination_map = cv2.bitwise_and(imagination_map,imagination_map,mask = mask2)
        cv2.imwrite(path_name_current_imagination_map_id_png, imagination_map)

    # save in a greyscale:
    imagination_map_cv2_grey = cv2.cvtColor(imagination_map, cv2.COLOR_BGR2GRAY)
    path_name_current_imagination_map_id_png_grey = 'imagination_map_grey'.join(img_name.split('costmap')) # imagination map = prediction map/costmap
    #cv2.imwrite(path_name_current_imagination_map_id_png_grey, imagination_map_cv2_grey) # for debugging
    # save in black (free) & white (occupied) scale to be able to visualize in rviz:
    imagination_map_cv2_black_white = np.zeros((imagination_map_cv2_grey.shape[0],imagination_map_cv2_grey.shape[1])) # (row,col) vs. (row,col,3)
    for i in range(imagination_map_cv2_grey.shape[0]):
        for j in range(imagination_map_cv2_grey.shape[1]):
            # - find the smallest value of the grey scale image
            # -> works only without having the circle filtering since then black=0 will be the smallest value and not the one we are searching for
            # -> as an alternative maybe normalize the grey image, so that the free space is indeed black!?!)
            #grey_factor = imagination_map_cv2_grey.min() # >0 not enough & > 100 is a further filtering that may not be wanted => imagination_map_cv2_grey.min() (=30!)
            grey_factor = 30
            if imagination_map_cv2_grey[i,j] > grey_factor: imagination_map_cv2_black_white[i,j] = 255
    path_name_current_imagination_map_id_png_black_white = 'imagination_map_black_white'.join(img_name.split('costmap')) # imagination map = prediction map/costmap
    #cv2.imwrite(path_name_current_imagination_map_id_png_black_white, imagination_map_cv2_black_white) # for debugging

    # continuously update from the local (for example 60x60) imagination costmap the global map!
    # -> only the observation lidar costmap could be get send directly from laser_scan_data.py
    row_big = x_max
    col_big = y_max
    white_ar = [255,255,255]
    grey_ar = [100,100,100]
    black_ar = [0,0,0]
    color_ar = [50,50,50] # dark grey

    # map_data comes from the subscribed topic with type OccupancyGrid
    local_costmap_resolution = map_data.info.resolution # 0.05 # width: 666 [px] * 0.05 (resolution) = 33.3 [m]
    #local_costmap_width = map_data.info.width # 60
    #local_costmap_height = map_data.info.height # 60
    local_costmap_width_default = map_data.info.width
    local_costmap_width = costmap_gt_range # could be 60/80/100/..
    local_costmap_height = costmap_gt_range # could be 60/80/100/..
    robot_position_x = map_data.info.origin.position.x
    robot_position_y = map_data.info.origin.position.y
    robot_orientation_z = map_data.info.origin.orientation.z # the orientation of the robot was not used, always z = 0 rad
    map_data_array = map_data.data # len() = 3600/6400/10000

    # Important: for rviz origin is on the bottom left, for an image is always on the top left; bottom left corner is for this map (-6,-6)! => corect the robot's position so that it is always positive
    # TODO: get params like resolution, origin etc. directly from the map yaml file instead of hard-coding them
    robot_position_x += abs(x_offset)
    robot_position_y += abs(y_offset)
    # (TODO X) Important: the 'correction' is needed to adjust the parameters of the default map of size 60x60 to another one of size like 80x80 or 100x100
    correction = int((local_costmap_width - local_costmap_width_default)/2) # int((100-60)/2)=20; int((80-60)/2)=10; int((60-60)/2)=0
    block_abs_width_left_rviz = int(robot_position_x / local_costmap_resolution) - correction
    block_abs_width_right_rviz = int((robot_position_x / local_costmap_resolution) + local_costmap_width) - correction
    block_abs_height_bottom_rviz = int(robot_position_y / local_costmap_resolution) - correction
    block_abs_height_top_rviz = int((robot_position_y / local_costmap_resolution) + local_costmap_height) - correction
    
    # TODO NEXT:
    # - 60x60, 80x80 or/and 100x100 ground truth map -> also the output of the model!
    # -> in order for the model to work with 80x80 gt map and return 80x80 map, the model should be first also trained that way (with lidar 60x60 and gt 80x80)!
    # - the not grey imagination costmap is only black!?
    # - visualize in rviz both the gt and the imagination costmap, with different colors when possible
    # -> https://answers.ros.org/question/280778/is-it-possible-to-change-color-scheme-for-rviz-map-display/
    
    # visualize the ground truth:
    #map_labels_npy = np.asarray(labels_test_npy) # shape = (1,1,60,60) -> (60,60)
    # visualize the imagination costmap: (should be black & white!) choose the model!
    ##map_labels_npy = np.asarray(current_model_labels_prediction_npy["occ_estimate"].detach()[0,0]) # shape = (1,1,60,60) -> (60,60) # it is colorful so it does not work, should be black & white!
    map_labels_npy = np.asarray(imagination_map_cv2_black_white) # shape = (1,1,60,60) -> (60,60)
    #map_reshaped = map_labels_npy.reshape(60,60)
    map_reshaped = map_labels_npy.reshape(costmap_gt_range,costmap_gt_range) # could be 60/80/100..
    # convert an OccupancyGrid pixel order to an image order -> flip the OccupancyGrid array around the x axis!
    map_reshaped = flip_img_x_axis(map_reshaped)

    # Important: do not allow any overlapping in the global image (because of small images wrongly rotated/flipped)
    # -> cause: it happens because of not paired images (for example a gt image alone) in between that mess everything up (a single X_ground_truth_map_part.png)
    # -> solution: publish all together both gt+costmap as an array[2] of occupancygrids!
    my_file = Path("imagination_map_global.png")
    if not(my_file.is_file()): # at the beginning when/if the file does not exist
        # idea 1: a completely black image
        map_init = np.zeros((row_big,col_big)) # size of the big map image # (row,col) vs. (row,col,3)
        # idea 2: /imagination_global should be = /map topic (TODO)
        ##rospy.Subscriber('/map', OccupancyGrid, callback_map)
        #map_init = cv2.imread("map_topic.png")
        # idea 3: /imagination_global should be = /global_costmap topic (TODO)
        #map_init = cv2.imread("map_global_costmap")
        temp_img = map_init
        temp_img_grey = map_init
        temp_img_one_color = map_init
    else:
        temp_img = cv2.imread("imagination_map_global.png")
        temp_img_grey = cv2.imread("imagination_map_global_grey.png") # should definetely include all laser scan points (in grey)
        temp_img_one_color = cv2.imread("imagination_map_global_one_color.png")
        for i in range(row_big):
            if i > block_abs_height_bottom_rviz and i <= block_abs_height_top_rviz:
                for j in range(col_big):
                    if j > block_abs_width_left_rviz and j <= block_abs_width_right_rviz:
                        if(map_reshaped[i-block_abs_height_bottom_rviz-1,j-block_abs_width_left_rviz-1].all()==-1):
                            temp_img[row_big-1-i,j] = 255 # unknown = white
                            temp_img_grey[row_big-1-i,j] = 255 # unknown = white
                            temp_img_one_color[row_big-1-i,j] = 255 # unknown = white
                            ##temp_img[i,j] = 255 # unknown = white
                            ##temp_img_grey[i,j] = 255 # unknown = white
                            ##temp_img_one_color[i,j] = 255 # unknown = white
                        else:
                            if temp_img_grey[row_big-1-i,j].all() == 0: # overwrite it only if it was before black (init status 'free'); if it was grey, leave it grey and do not overwrite it with black
                                if map_reshaped[i-block_abs_height_bottom_rviz-1,j-block_abs_width_left_rviz-1].all() > 0:
                                    temp_img_grey[row_big-1-i,j] = grey_ar # black = free; grey = occupied
                                    #temp_img_grey[i,j] = grey_ar # black = free; grey = occupied
                            if temp_img[row_big-1-i,j].all() == 0: # overwrite it only if it was before black (init status 'free'); if it was grey, leave it grey and do not overwrite it with black
                                if map_reshaped[i-block_abs_height_bottom_rviz-1,j-block_abs_width_left_rviz-1].all() > 0:
                                    temp_img[row_big-1-i,j] = white_ar
                                    # (TODO) get_color_from_id() does not work, since the imagination image is black and white and has been trained on with black and white images => black=0=free, white=255=occupied
                                    #temp_img[row_big-1-i,j] = get_color_from_id(map_reshaped[i-block_abs_height_bottom_rviz-1,j-block_abs_width_left_rviz-1])
                                    ##temp_img[i,j] = get_color_from_id(map_reshaped[i-block_abs_height_bottom_rviz-1,j-block_abs_width_left_rviz-1])
                            if temp_img_one_color[row_big-1-i,j].all() == 0: # overwrite it only if it was before black (init status 'free'); if it was grey, leave it grey and do not overwrite it with black
                                if map_reshaped[i-block_abs_height_bottom_rviz-1,j-block_abs_width_left_rviz-1].all() > 0:
                                    temp_img_one_color[row_big-1-i,j] = color_ar # black = free; color = occupied
                                    ##temp_img_one_color[i,j] = grey_ar # black = free; color = occupied

    #cv2.imshow("map_imagination_costmap", temp_img)
    # temp imagination global image: (for debugging the progress of updating the global imagination)
    #cv2.imwrite('imagination_map_global'.join(img_name.split('costmap')), temp_img)
    #cv2.imwrite('imagination_map_global_grey'.join(img_name.split('costmap')), temp_img_grey)
    # final imagination global image:
    cv2.imwrite("imagination_map_global.png", temp_img)
    cv2.imwrite("imagination_map_global_grey.png", temp_img_grey)
    cv2.imwrite("imagination_map_global_one_color.png", temp_img_one_color)
    #cv2.waitKey(0)

    # TODO NEXT: update the occupancy grid of the rviz map with the imagination?!
    # -> so that the robot immediately plans with the imagination as obstacles
    # -> set FixedFrame in rviz to sth else then /map?
    # Important - Question: the lidar (local costmap) don't have to be updated with the imagination data, because then the laser will scan sth that it hasn't been trained on, correct?
    # -> & under the tables and chair with teleoperation the robot also could be driven, so important is that move_base recognizes the imagination as obstacle so that the robot does not draw over and the local planner plan to avoid it, correct?
    #pub1 = rospy.Publisher("/map", OccupancyGrid, queue_size=10) # (521, 666) # publish the whole time the new costmap

    # publish the imagination costmap to a topic and visualize in rviz (include the topics also directly in the .rviz file)
    # (as an alternative in rviz for Map the default topic /map could be changed to /imagination_global)
    pub2 = rospy.Publisher("/imagination_global", OccupancyGrid, queue_size=10) # (521, 666)
    grid = OccupancyGrid()
    grid.header.seq = 0
    grid.header.stamp = rospy.Time.now()
    grid.header.frame_id = ""
    # get info directly from the subscribed topic, but change width, hight and position!
    grid.info.width = temp_img_grey.shape[1] # 666 !
    grid.info.height = temp_img_grey.shape[0] # 521 !
    grid.info.resolution = map_data.info.resolution
    grid.info.map_load_time = map_data.info.map_load_time
    grid.info.origin.orientation = map_data.info.origin.orientation
    grid.info.origin.position.x = x_offset # ! # -6
    grid.info.origin.position.y = y_offset # ! # -6
    grid.info.origin.position.z = 0
    map_array = []
    i = temp_img_grey.shape[0] - 1
    while i >=0:
        for j in range(temp_img_grey.shape[1]):
            if len(temp_img_grey.shape) < 3: map_array.append(0)
            else:
                #map_array.append(temp_img_grey[i,j])
                BGR_color = [temp_img_grey[i, j, 0], temp_img_grey[i, j, 1], temp_img_grey[i, j, 2]]
                if(BGR_color==black_ar): # black
                    map_array.append(0) # 0 = black = free
                elif(BGR_color==white_ar): # white
                    map_array.append(-1) # -1 = white = unknown
                else: # grey
                    #map_array.append(100) # 100 = grey = occupied
                    map_array.append(101) # different color then grey, but still occupied!?
                    # with -1 or 50 for example: colored differently in rviz, costmap not changed, move_base drives over (as if free)
                    # with 101: as if it was 100: costmap overlapped with gt, move_base does not drive over
        i -= 1
    grid.data = map_array
    #pub1.publish(grid)
    pub2.publish(grid)
    # TODO Important: update /map? Be careful!, because then the local_costmap changes too and the model can not deal with such kind of data!
    ##time.sleep(2) # sleep() in between to give time to the planner to plan again a new path?
    pub3 = rospy.Publisher("/map", OccupancyGrid, queue_size=10)
    grid.header.frame_id = "map"
    pub3.publish(grid)
    ##time.sleep(2)

    # Important!: subscribing to the topic, updates the global_costmap image, otherwise it will remain showing only its initial state!!!
    rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, callback_global_costmap)

    # cut temp_img_grey to the current, local 60x60/80x80/100x100 block (with a variable size):
    temp_img_grey_part = temp_img_grey[row_big-1-block_abs_height_top_rviz+1:row_big-1-block_abs_height_bottom_rviz+1, block_abs_width_left_rviz+1:block_abs_width_right_rviz+1]
    pub = rospy.Publisher("/imagination", OccupancyGrid, queue_size=10) # (60,60)/(80,80)/(100,100)
    grid2 = OccupancyGrid()
    grid2.header.seq = 0
    grid2.header.stamp = rospy.Time.now()
    grid2.header.frame_id = ""
    grid2.info = map_data.info # get info per default from the subscribed topic and then correct some of the parameters
    grid2.info.width = costmap_gt_range # correct the width, could be 60/80/100/..
    grid2.info.height = costmap_gt_range # correct the height, could be 60/80/100/..
    # (TODO X) Important: 'correction' factor also here, but in meter, not in pixels => correction*local_costmap_resolution (for exaxmple: 20px*0.05=1m)
    grid2.info.origin.position.x = map_data.info.origin.position.x - correction*local_costmap_resolution
    grid2.info.origin.position.y = map_data.info.origin.position.y - correction*local_costmap_resolution
    map_array2 = []
    print(temp_img_grey_part.shape)
    i = temp_img_grey_part.shape[0] - 1
    while i >=0:
        for j in range(temp_img_grey_part.shape[1]):
            if len(temp_img_grey_part.shape) < 3: map_array2.append(0)
            else:
                #map_array.append(temp_img_grey[i,j])
                BGR_color = [temp_img_grey_part[i, j, 0], temp_img_grey_part[i, j, 1], temp_img_grey_part[i, j, 2]]
                if(BGR_color==black_ar): # black
                    map_array2.append(0) # 0 = black = free
                elif(BGR_color==white_ar): # white
                    map_array2.append(-1) # -1 = white = unknown
                else: # grey
                    map_array2.append(100) # 100 = grey = occupied
        i -= 1
    grid2.data = map_array2
    pub.publish(grid2)
    #print('*//*//*')
    #print(temp_img_grey.shape) # (521, 666, 3)
    #print(len(map_array)) # 346986 (= 521*666)
    #print(temp_img_grey_part.shape) # (60, 60, 3)
    #print(len(map_array2)) # 3600
    #print('*//*//*')
    
    # TODO: this new imagination costmap image (the prediction) should also be checked while deleting the black images!

def callback_global_costmap(map_data): # TODO: it does not update itself when the robots is moving, even though it does in rviz!?
    #global map_resolution, x_offset, y_offset, x_max, y_max
    #x_max = 521
    #y_max = 666
    map_resolution, x_offset, y_offset, x_max, y_max = get_map_parameters()
    
    print('GLOBAL COSTMAP: ' + str(len(map_data.data))) # 346986 (the same length as the one from /map), but different info: it updates globally with the info from the obstacles while the robot is moving)
    #print(map_data) # consists of a header, metadata info and a data array, where 0 = free, 100 = occupied, -1 = unknown # whiter pixels are free, blacker pixels are occupied, and pixels in between are unknown
    map_data_array = asarray([map_data.data])
    #savetxt('global_costmap_data.csv', map_data_array, delimiter=',') # will be saved in folder $HOME\.ros
    free_amount = 0
    unknown_amount = 0
    ocupied_amount = 0
    for i in map_data_array[0]:
        if i == 0:
            free_amount += 1
        if i == -1:
            unknown_amount += 1
        if i == 100:
            ocupied_amount += 1
    print("MAP - FREE: " + str(free_amount) + ", UNKNOWN: " + str(unknown_amount) + ", OCCUPIED: " + str(ocupied_amount)) # should be updating

    # save the map as a grey image (black = free; grey = occupied; unknown = white):
    map_data_array2 = np.array(map_data.data) # array of size 346986
    map_reshaped = map_data_array2.reshape((x_max,y_max))
    print(map_reshaped)
    row,col = map_reshaped.shape
    temp = np.zeros((row,col)) # (row,col) vs. (row,col,3)
    # the occupancy grid is in row-major order, starting with (0,0); our (0,0) is in the left down corner; for an image it is the upper left corner => mirror the pixels regarding the x axis to be right
    for i in range(row):
        for j in range(col):
            if(map_reshaped[i,j]==-1):
                temp[row-1-i,j]=255 # unknown = white
            else:
                temp[row-1-i,j]=map_reshaped[i,j] # black = free; grey = occupied
    #cv2.imshow("map_global_costmap", temp)
    cv2.imwrite("map_global_costmap.png", temp) # will be saved in folder $HOME\.ros
    #cv2.waitKey(0)

def callback_map(map_data):
    #global map_resolution, x_offset, y_offset, x_max, y_max
    #x_max = 521
    #y_max = 666
    map_resolution, x_offset, y_offset, x_max, y_max = get_map_parameters()
    
    # TODO: wait for the obstacles to be spawned!?
    #print(map_data) # consists of a header, metadata info and a data array, where 0 = free, 100 = occupied, -1 = unknown # whiter pixels are free, blacker pixels are occupied, and pixels in between are unknown
    map_data_array = asarray([map_data.data])
    #savetxt('map_data.csv', map_data_array, delimiter=',') # will be saved in folder $HOME\.ros
    free_amount = 0
    unknown_amount = 0
    ocupied_amount = 0
    for i in map_data_array[0]:
        if i == 0:
            free_amount += 1
        if i == -1:
            unknown_amount += 1
        if i == 100:
            ocupied_amount += 1
    #print("MAP - FREE: " + str(free_amount) + ", UNKNOWN: " + str(unknown_amount) + ", OCCUPIED: " + str(ocupied_amount)) # FREE: 278156, UNKNOWN: 2134, OCCUPIED: 66696

    # save the map as a grey image (black = free; grey = occupied; unknown = white):
    map_data_array2 = np.array(map_data.data) # array of size 346986
    map_reshaped = map_data_array2.reshape((x_max,y_max))
    print(map_reshaped)
    row,col = map_reshaped.shape
    temp = np.zeros((row,col)) # (row,col) vs. (row,col,3)
    # the occupancy grid is in row-major order, starting with (0,0); our (0,0) is in the left down corner; for an image it is the upper left corner => mirror the pixels regarding the x axis to be right
    for i in range(row):
        for j in range(col):
            if(map_reshaped[i,j]==-1):
                temp[row-1-i,j]=255 # unknown = white
            else:
                temp[row-1-i,j]=map_reshaped[i,j] # black = free; grey = occupied
    #cv2.imshow("map_topic", temp)
    cv2.imwrite("map_topic.png", temp) # will be saved in folder $HOME\.ros
    #cv2.waitKey(0)

def flip_img_x_axis(img):
    row,col = img.shape
    img_flipped = np.zeros((row,col)) # (row,col) vs. (row,col,3)
    for i in range(row):
        for j in range(col):
            img_flipped[row-1-i,j] = img[i,j]
    return img_flipped

def callback_pose(data):
    global move_base_goal, goal_global
    move_base_goal.target_pose = data
    goal_global = data

def callback_odom(data):
    global position_global, orientation_global
    position_global = data.pose.pose.position
    orientation_global = data.pose.pose.orientation

# TODO: when the robot gets stuck, multiple images that are the same are taken
# TODO: on the local costmap images there are grey pixels on the right and upper obstacle's borders -> further neighbours were checked!?!
def save_img(data, img_name): # data.data[costmap, gt]
    global temp_time, time_start, img_sec, start_path, final_path
    temp_time = rospy.get_rostime().secs

    #rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback_pose)
    #rospy.Subscriber("/odom", Odometry, callback_odom)
    if start_path == 0 and final_path == 0: # without the path before the start and the one after the last goal back to the start
        # TODO: have not one of 60x60, 80x80, 100x100, but all of them
        # -> change the file name from _ground_truth_map to ground_truth_map_60x60 etc.
        # --> correct the part with deleting the paired black images
        if (img_name == "pair_part_60") or (img_name == "pair_part_80") or (img_name == "pair_part_100"):
            # TODO: make a difference if the script is used only for collecting data or also for navigation with imagination
            # -> if only for collecting data, then the function imagination() shouldn't be called and sync_robot_step should be 0 (turned off)
            if rospy.get_param('~imagination') == "no": imagination_bool = 0
            else: imagination_bool = 1

            costmap_gt_range = int(img_name.split("_")[2])
            i = 0
        #    while temp_time >= time_start + i*img_sec: # this check is now in laser_scan_data.py in callback_local_costmap() !
        #        if temp_time == time_start + i*img_sec:
            sync_robot_step = imagination_bool # 0=off/1=on # TODO X
            # TODO NEXT: make the robot's movement slower the whole time!?
            # TODO NEXT: make the /cmd_vel has the same speed m/sec, as the needed time in sec for laser scan based on the path, so that the robot never stops, but just slows down!?
            # -> laser scan is taken every three seconds = img_sec (make img_Sec bigger then it should be faster!?)
            # --> even tough it is every 3 sec, the laser scan updates by every change! there should be also only every 3 sec!?!
            # -> at the beginning vel=0.4m/sec and it is fast
            # -> let's say that the laser scan needs around one second to generate the map
            if sync_robot_step == 1:
                # TODO NEXT: pause the robot's movement!? (synchronization)
                # -> the robot does stop, BUT the goal is sometimes updating to the next one, without being reached!?!
                # -> the indexes are not (always) in interval of 3?? -> with idea1.2 in interval of 6 (rarely 9)
                # idea0?
                rospy.Subscriber("/cmd_vel", Twist, callback_move)
                pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
                speed = Twist()
                speed.linear.x = 0.0
                speed.linear.y = 0.0
                speed.linear.z = 0.0
                speed.angular.x = 0.0
                speed.angular.y = 0.0
                speed.angular.z = 0.0
                #pub.publish(speed)
                # idea1: subscribe to odom to get the current pos and in the callback function set the goal = current pos
                rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback_pose)
                rospy.Subscriber("/odom", Odometry, callback_odom)
                pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
                global position_global, orientation_global
                goal = PoseStamped()
                goal.header.seq = 0
                goal.header.frame_id = "map"
                goal.header.stamp = rospy.Time.now()
                goal.pose.position = position_global
                goal.pose.orientation = orientation_global
                pub_goal.publish(goal) # publish the current pos as goal to temporary stop the goal until done
                # 
                #actionlib_goal = MoveBaseGoal()
                #actionlib_goal.target_pose = goal
                #client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
                #client.wait_for_server()
                #client.send_goal(actionlib_goal)
                #wait = client.wait_for_result()
                #if wait: return client.get_result()
                # idea2: use actionlib?
            #    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            #    client.cancel_goal()
                # idea
                #goal3 = MoveBaseGoal()
                #goal3.target_pose = goal
                #client.send_goal(goal3)
                # 
                #movebase_client(position_global.x, position_global.y, pub_goal, 0, 0)
            
            #costmap_image = np.array(data.data[0].data).reshape(60, 60)
            costmap_image = np.array(data.data[0].data).reshape(costmap_gt_range, costmap_gt_range) # could be 60/80/100..
            #gt_image = np.array(data.data[1].data).reshape(60, 60)
            gt_image = np.array(data.data[1].data).reshape(costmap_gt_range, costmap_gt_range) # could be 60/80/100..
            path_name_costmap = "training/" + str(rospy.get_rostime().secs) + "_costmap_part.png"
            path_name_gt = "training/" + str(rospy.get_rostime().secs) + "_ground_truth_map_part.png"
            # the saved part images themself should be also flipped around the x axis -> convert an OccupancyGrid pixel order to an image order
            #cv2.imwrite(path_name_costmap, flip_img_x_axis(costmap_image))
            #cv2.imwrite(path_name_gt, flip_img_x_axis(gt_image))
            # Important: in addition, to have it in color, get_color_from_id() should be used:
            cv2.imwrite(path_name_costmap, get_color_from_id_array(flip_img_x_axis(costmap_image)))
            cv2.imwrite(path_name_gt, get_color_from_id_array(flip_img_x_axis(gt_image)))
            if imagination_bool == 1:
                imagination(data.data[0], path_name_costmap, costmap_gt_range) # TODO X
                print('The predicted imagination image has been created!')
            # TODO:
            pub_counter = rospy.Publisher("/imagination_counter", String, queue_size=10)
            global imagination_counter
            imagination_counter += 1
            #print(imagination_counter)
            pub_counter.publish(str(imagination_counter))

            if sync_robot_step == 1:
                # TODO NEXT: continue the robot's movement!?
                # idea0?
                rospy.Subscriber("/cmd_vel", Twist, callback_move)
                pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
                speed = Twist()
                global global_speed
                speed = global_speed
                #pub.publish(speed)
                # idea1
                pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
                global goal_global, sub_goal
                #print('B: ' + str(sub_goal)) # debugging
                #pub_goal.publish(goal_global) # publish back the same goal
                # idea1.2 (better)
                goal.header.seq = 0
                goal.header.frame_id = "map"
                goal.header.stamp = rospy.Time.now()
                goal.pose.position.x = sub_goal[0]
                goal.pose.position.y = sub_goal[1]
                goal.pose.position.z = 0.0
                goal.pose.orientation = orientation_global
                pub_goal.publish(goal)
                # 
                #actionlib_goal = MoveBaseGoal()
                #actionlib_goal.target_pose = goal_global
                #client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
                #client.wait_for_server()
                #client.send_goal(actionlib_goal)
                #wait = client.wait_for_result()
                #if wait: return client.get_result()
                # idea2
                #global move_base_goal
                #client.send_goal(move_base_goal)
                # idea
                #client.cancel_goal()
                #goal2 = MoveBaseGoal()
                #goal2.target_pose = goal
                #client.send_goal(goal2)
                # 
                #movebase_client(sub_goal[0], sub_goal[1], pub_goal, 0, 0)
        #        i += 1
        else:
            # subscribe only every img_sec seconds (for example every 1, 2 or 3 seconds):
            # TODO: is the rospy time the same as the one in the simulation / as the one in real world?
            i = 0
            while temp_time >= time_start + i*img_sec:
                if temp_time == time_start + i*img_sec:
                    # converting ROS image messages to OpenCV images
                    bridge = CvBridge()
#                    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
                    # convert back to opencv image
                    ### new idea: (1) list -> np.array; (2) (10800,,) -> (60,60,3)
                    #cv_image = np.array(data.data).reshape(60, 60, 3) # TODO - for ListInt
                    costmap_image = np.array(data.data).reshape(60, 60) # TODO - for OccupancyGrid
                    # save the image in the folder 'training' always with a different name (add _id or timestamp at the end for example)
                    # for now save also completely black images (even though they are not important for the training), they will be filtered out at the end by a separate function
                    path_name = "training/" + str(rospy.get_rostime().secs) + "_" + img_name + ".png"
                    #cv2.imwrite(path_name, cv_image) # TODO - for ListInt
                    # TODO: Convert an OccupancyGrid pixel order to an image order -> flip the OccupancyGrid array around the x axis!
                    cv2.imwrite(path_name, costmap_image) # TODO - for OccupancyGrid
                    #temp = cv2.imread(path_name)
                    #cv_image2 = cv2.cvtColor(temp, cv2.COLOR_BGR2RGB)
                    #cv2.imwrite("training/" + str(rospy.get_rostime().secs) + "_" + img_name + "TEST.png", cv_image2)
                    if img_name == "costmap_part": # TODO NEXT
                        imagination(data, path_name, 60)
                        print('The predicted imagination image has been created!')
                    break
                i += 1

def callback_costmap_temp(data):
    save_img(data, "costmap_part")

def callback_ground_truth_map_temp(data):
    save_img(data, "ground_truth_map_part")

def callback_pair_temp_60x60(data):
    save_img(data, "pair_part_60")

def callback_pair_temp_80x80(data):
    save_img(data, "pair_part_80")

def callback_pair_temp_100x100(data):
    save_img(data, "pair_part_100")

def callback_obstacles_map_temp(data):
    save_img(data, "obstacles_map_part")

def callback_counter(data):
    global imagination_counter
    imagination_counter = int(data.data)

def callback_timer(data):
    global timer_done
    timer_done = data.data

if __name__ == '__main__':
    rospy.init_node('reach_goal', anonymous=True)

    rospy.Subscriber("/costmap_timer_done", String, callback_timer)
    rospy.Subscriber('/map', OccupancyGrid, callback_map)
    #rospy.Subscriber("/cmd_vel", Twist, callback_move) # TODO NEXT?
    #rospy.Subscriber("/odom", Odometry, callback_move) # TODO NEXT?
    #rospy.Subscriber('/imagination_counter', String, callback_counter)

    # goal_publisher() # the robot moves to the goal, but does not avoid obstacles
    # goal_publisher_move_base() # the robot doeas not move (theoretically it should avoid the obstacles)
    # goal_publisher_move_base_client() # the robot moves to goal and avoids the obstacles
    training_script()
    #delete_empty_images_get_raw_data()

    #rospy.spin()

# %%
