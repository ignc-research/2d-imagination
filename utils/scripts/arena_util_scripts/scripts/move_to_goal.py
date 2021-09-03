#!/usr/bin/env python
import rospy, rosparam
from math import atan2
import yaml, json
from nav_msgs.msg import Odometry, OccupancyGrid
from tf.transformations import euler_from_quaternion # to make it work with python3 do: cd $HOME/geometry2_ws $ source devel/setup.bash $ cd $HOME/catkin_ws_ma $ source devel/setup.bash
from geometry_msgs.msg import Point, Twist, PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
import cv2
import os
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os, glob
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from numpy import asarray, savetxt

x_global = 0.0
y_global = 0.0 
theta_global = 0.0

time_start = 0.0
temp_time = 0.0
img_sec = 1 # default

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
    pub = rospy.Publisher("/cmd_vel", Twist)
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
    pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal)
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

    pose_stamped = PoseStamped()
    pose_stamped = goal.target_pose
    goal_pub.publish(pose_stamped)

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    client.send_goal(goal)
    wait = client.wait_for_result()
    if wait:
        return client.get_result()
    #else:
    #    rospy.logerr("Action server not available!")
    #    rospy.signal_shutdown("Action server not available!")

def goal_publisher_move_base_client():
    goal_status_sub = rospy.Subscriber("/move_base/status", GoalStatusArray, goal_status_sub_callback)
    goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped) # TODO: shows the goals (their position and orientation with an arrow), but starting with the second goal?
    # TODO: control the speed, overwrite only the linear.x value of the /cmd_vel topic!?

    # move directly to an example position (for debugging purposes)
    #movebase_client(5, 0, goal_pub) # move just slightly
    #movebase_client(13.8938770294, 4.7637386322, goal_pub) # go in between the tables

    # move zig-zag from left to right
    # the robot automatically waits until the current goal has been reached before pursuing the next one
    x_min = -3
    x_max = 25
    y_min = 0
    y_max = 17.5
    y_step = 2.5
    y_temp = y_min
    while y_temp <= y_max - y_step:
        movebase_client(x_max, y_temp, goal_pub)
        y_temp += y_step
        movebase_client(x_min, y_temp, goal_pub)

    # TODO DEBUGGING gmapping (for scenario 1):
    #movebase_client(5, -1, goal_pub)
    #movebase_client(24, -1, goal_pub)
    #movebase_client(24, 17, goal_pub)
    #movebase_client(5, 17, goal_pub)
    #movebase_client(5, 0, goal_pub)

    # TODO DEBUGGING testing robot's velocity:
    #movebase_client(5, 0, goal_pub)
    #movebase_client(5, 5, goal_pub)
    #movebase_client(0, 5, goal_pub)
    #movebase_client(0, 0, goal_pub)
    #movebase_client(5, 0, goal_pub)

def training_script():
    goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped) # TODO: shows the goals (their position and orientation with an arrow), but starting with the second goal?
    
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
    with open("/home/m-yordanova/catkin_ws_ma/src/arena-rosnav/simulator_setup/training/scenario1_idea2.yaml", 'r') as stream:
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
    with open("/home/m-yordanova/catkin_ws_ma/src/arena-rosnav/simulator_setup/training/scenario1.json", 'r') as stream:
    #with open("$(find simulator_setup)/training/test.json", 'r') as stream:
        doc = json.load(stream)
        print('json file content:\n' + str(doc) + '\n')
        # example: {u'num_images': 101, ...}
        
        # Important: in the gui the x-y origin is in the top left corner, but for us it is in the bottom left corner, but
        # all points in the json file already are correctly scaled and transformed, so they are in meters, not in pixels and ready to be used with move_base without any change!
        # that is why we also do not need the parameters 'resolution' and 'origin'
        map = doc['map_path'] # TODO: with or without a path? (in the gui the yaml file for the map should be selected)
        
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
        #pub.publish(grid)

        global time_start, temp_time, img_sec
        # TODO: amount of images per path vs. images per second -> images per second is easier since it could be the same for all paths in a scenario even if the paths have a different length
        num_images = doc['num_images'] # number of images per minute so that we always have an integer value; max: 1 img per millisecond, min: 1 img per minute
        img_sec = int(1.0/(num_images/60.0)) # use =3 for debugging
        for path in doc['robot_paths']:
            initial_pos = path['initial_pos']
            movebase_client(initial_pos[0], initial_pos[1], goal_pub) # first move to the initial pose
            # rospy.Time.now() vs. rospy.get_time() vs. rospy.get_rostime().secs
            #time_start = rospy.get_time() # Get the current time in float seconds.
            time_start = rospy.get_rostime().secs # get only the seconds (it's enough?), then the number do not have to be converted to integer while comparing the times
            temp_time = rospy.get_rostime().secs
            # get the pair images costmap_part-ground_truth_map_part by subscribing to the new topics from the 'scan_values' node
            sub_costmap = rospy.Subscriber("costmap_temp", Image, callback_costmap_temp)
            sub_ground_truth_map = rospy.Subscriber("ground_truth_map_temp", Image, callback_ground_truth_map_temp)
            #sub_obstacles_map = rospy.Subscriber("obstacles_map_temp", Image, callback_obstacles_map_temp)
            for subgoal in path['subgoals']:
                # the robot automatically waits until the current goal has been reached before pursuing the next one
                movebase_client(subgoal[0], subgoal[1], goal_pub)
                # TODO NEXT: tune the robot's orientation at each subgoal?! or just set a bigger radius around the goal that passes as 'goal reached'
                # -> change parameters xy_goal_tolerance and yaw_goal_tolerance to a bigger values!? in /arena-rosnav/arena_navigation/arena_local_planner/model_based/conventional/config/base_local_planner_params.yaml but no change visible!?
            # unsubscribe until the next initial_pos has been reached and then subscribe again:
            # but since computation of loca costmap takes time, we may lose the last data if we unsubscribe, so better don't unsubscribe here and spend more time at the end deleting the black images
            #sub_costmap.unregister()
            #sub_ground_truth_map.unregister()
            #sub_obstacles_map.unregister()
        # TODO: since the laser node computes slowly, the first couple of images in the training folder are black and the couple of images of the path are missing!
        movebase_client(0, 0, goal_pub) # maybe at the end move again to origin (0,0) to get a chance to collect the rest of the local_costmap data
        delete_empty_images_get_raw_data()

def delete_empty_images_get_raw_data():
    # 1) open each img in the training folder and if it is complety black delete it, since they are not important/useful for the training
    # TODO NEXT: it could happen that the costmap is empty, but the gt pair is not, so either delete both or do not delete either of them!?!
    # -> a quick check is that the number of images should be an odd number!
    # -> it is also better to run this function separately at the end once it was made sure that the image pairs are fine and final!
    # 2) get the raw data from every image and save it into a npy file
    print('Preparing the raw training data ...')
    path = './training'
    container_costmap_color = [] # TODO: one for ground truth data and one for costmap data (both arrays should have the same order!)
    container_ground_truth_color = []
    container_costmap_id = []
    container_ground_truth_id = []
    #container_dict = [] # TODO: use a dict instead, so that there is only one npz file (just appending costmap, gt, costmap, gt etc. won't work, since npz do not keep the order!?)
    #dict = {'costmap' : 'asarray(filename1)', 'ground_truth' : 'asarray(filename2)'}
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
    
    for filename in glob.glob(os.path.join(path, '*.png')):
        img = cv2.imread(filename)
        all_black = True
        black_ar = [0,0,0]
        img_id = np.zeros((img.shape[0],img.shape[1]))
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
                    if int(elem['color'][0]*255) == BGR_color[2] and int(elem['color'][1]*255) == BGR_color[1] and int(elem['color'][2]*255) == BGR_color[0]:
                        id_temp = elem['id']
                        break
                img_id[i,j] = id_temp

        if all_black:
            os.remove(filename) # delete the image
        else:
            img_ar = asarray(img) # img_ar.shape = (60,60) # rgb color
            img_id_ar = asarray(img_id) # img_id_ar.shape = (60,60) # id
            #print(img_ar) # [[ 0  0  0] [ 0  0  0] ..., [ 0  0  0] [ 0  0  0]]] # rgb color
            #print(img_id_ar) # [[ 0. 0. ..., 0. 0.] [ 0. 0. ..., 0. 0.] ..., [ 0. 0. ..., 0. 0.]] # id
            if ''.join(filename.split('costmap')) != filename: # costmap
                container_costmap_color.append(img_ar)
                container_costmap_id.append(img_id_ar)
            else: # ground truth
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

    print('DONE!')

# TODO: when the robot gets stuck, multiple images that are the same are taken
# TODO: on the local costmap images there are grey pixels on the right and upper obstacle's borders -> further neighbours were checked!?!
def save_img(data, img_name):
    global temp_time, time_start, img_sec
    temp_time = rospy.get_rostime().secs
    # subscribe only every img_sec seconds (for example every 1, 2 or 3 seconds):
    # TODO: is the rospy time the same as the one in the simulation / as the one in real world?
    i = 0
    while temp_time >= time_start + i*img_sec:
        if temp_time == time_start + i*img_sec:
            # converting ROS image messages to OpenCV images
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            # convert back to opencv image and same the image in the folder 'training' always with a different name (add _id or timestamp at the end for example)
            # for now save also completely black images (even though they are not important for the training), they will be filtered out at the end by a separate function
            cv2.imwrite("training/" + str(rospy.get_rostime().secs) + "_" + img_name + ".png", cv_image)
            break
        i += 1

def callback_costmap_temp(data):
    save_img(data, "costmap_part")

def callback_ground_truth_map_temp(data):
    save_img(data, "ground_truth_map_part")

def callback_obstacles_map_temp(data):
    save_img(data, "obstacles_map_part")

if __name__ == '__main__':
    rospy.init_node('reach_goal', anonymous=True)
    # goal_publisher() # the robot moves to the goal, but does not avoid obstacles
    # goal_publisher_move_base() # the robot doeas not move (theoretically it should avoid the obstacles)
    #goal_publisher_move_base_client() # the robot moves to goal and avoids the obstacles
    training_script()
    #delete_empty_images_get_raw_data()
