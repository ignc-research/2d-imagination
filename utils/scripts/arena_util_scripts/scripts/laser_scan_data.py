#!/usr/bin/env python
from ctypes import Array
from array import ArrayType
import rospy
import os
import math
import time
import rospkg
import cv2
import numpy as np
import matplotlib.pyplot as plt
import PIL
from pathlib import Path
from numpy import asarray, savetxt
from collections import namedtuple
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg
from visualization_msgs.msg import MarkerArray
from map_msgs.msg import OccupancyGridUpdate
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from arena_plan_msgs.msg import IntList, ListIntList, ListListIntList, ListOccupancyGrid
import threading
from std_msgs.msg import String
import yaml

lp = lg.LaserProjection()
read_laser_scan_info = 1

robot_pos_x = 0.0
robot_pos_y = 0.0
robot_angle_yaw_grad = 0.0

map_topic_2darray = [] # 521x666

time_start = 0.0
temp_time = 0.0
img_sec = 1 # default

#map_resolution = 0.0
#x_offset = 0
#y_offset = 0
#x_max = 0
#y_max = 0

def calculate_avg_x_y_value(point_generator):
    sum_temp_x = 0.0
    num_temp_x = 0
    sum_temp_y = 0.0
    num_temp_y = 0
    for point in point_generator:
        #print(point)
        if not math.isnan(point[0]):
            sum_temp_x += point[0]
            num_temp_x += 1
        if not math.isnan(point[1]):
            sum_temp_y += point[1]
            num_temp_y += 1
    print('avg x value: ' + str(sum_temp_x/num_temp_x)) # calculate avg x value
    print('avg y value: ' + str(sum_temp_y/num_temp_y)) # calculate avg y value

def viualize_laser_scan_data(point_list, r_pos_x, r_pos_y, r_angle):
    #global map_resolution, x_offset, y_offset, x_max, y_max
    #map_resolution = 0.05
    #x_offset = -6
    #y_offset = -6
    #x_max = 521
    #y_max = 666
    map_resolution, x_offset, y_offset, x_max, y_max = get_map_parameters()

    # (TODO) take data only every ten seconds for example
    point_list_abs = []
    AbsPoint = namedtuple("AbsPoint", "x y")
    point_x_max = point_y_max = 0 # find the biggest x and y -> column and row end
    point_x_min = point_y_min = 1000 # find the smallest x and y -> column and row start
    for point in point_list:
        x_abs = point.x + r_pos_x # TODO: check!
        y_abs = point.y + r_pos_y
        x_px = int((x_abs - x_offset) / map_resolution)
        y_px = int((y_abs - y_offset) / map_resolution)
        point_list_abs.append(AbsPoint(x_px, y_px)) # ({'x': x_px, 'y': y_px})
        if x_px > point_x_max: point_x_max = x_px
        if y_px > point_y_max: point_y_max = y_px
        if x_px < point_x_min: point_x_min = x_px
        if y_px < point_y_min: point_y_min = y_px
    print(point_list_abs[len(point_list_abs)/2]) # for debugging
    row = point_y_max - point_y_min
    col = point_x_max - point_x_min
    print('ROW(' + str(point_y_min) + ',' + str(point_y_max) + ') & COL(' + str(point_x_min) + ',' + str(point_x_max) + ')')
    # TODO: sometimes the colums are > 700, even tought max 665/666

    row_big = x_max
    col_big = y_max

    # for the map and the robot the origin (0,0) is in the left down corner; for an image it is always the upper left corner => mirror the pixels regarding the x axis to be right
    temp_small = np.zeros((row,col)) # (row,col) vs. (row,col,3)
    for i in range(point_y_min, point_y_max+1):
        for j in range(point_x_min, point_x_max+1):
            for point in point_list_abs:
                if point.x == j and point.y == i:
                    temp_small[row-1-(i-point_y_min),j-point_x_min-1] = 100 #  100 = grey = occupied; 0 = black = free
                    #temp_small[row-1-i,j] = 100 #  100 = grey = occupied; 0 = black = free
    cv2.imwrite("map_laser_scan_part.png", temp_small) # will be saved in folder $HOME\.ros

    # TODO NEXT - the visualization is not correct yet:
    # - maybe here is the angle important (not like with the local costmap); check the border values!
    # - the point cloud coordinates are relative to the robot coordinates? where is x and where y axis?
    # - the calculations and iteration are happening maybe just too slowly, the calculated min-max borders to not match anymore with the current laser scan area etc.!?
    # - sometimes 'point_x_min' and 'point_y_min' have a negative value!?

    # TODO: rotate the whole part image with robot_angle_yaw_grad=r_angle and then add it on top of the big one
    center_part_image_x = col/2
    center_part_image_y = row/2
    #diagonal_part_image = int(math.sqrt(math.pow(row,2)) + math.sqrt(math.pow(col,2)))
    print('ROTATING with an angle = ' + str(r_angle))
    M = cv2.getRotationMatrix2D((center_part_image_x,row_big-1-center_part_image_y),r_angle,1) # create the transformation matrix (center, angle, scale); important: (0,0) top left corner
    dst = cv2.warpAffine(temp_small,M,(col,row))
    cv2.imwrite("map_laser_scan_part_test.png", dst)

    # preferably there should be already a black image with this name and with the right dimensions in the $HOME\.ros folder,
    # if not - an image will be generated, but this takes time and in between the robot moves => some laser scan data can be missed and not be visualized
    my_file = Path("map_laser_scan.png")
    if not(my_file.is_file()): # if the file does not exist
        temp = np.zeros((row_big,col_big)) # (row,col) vs. (row,col,3)
    else:
        temp = cv2.imread("map_laser_scan.png")
        for i in range(row_big): # row_big - 0:521-1
            if i > point_y_min and i <= point_y_max: # TODO: check!
                for j in range(col_big): # col_big - 0:666-1
                    if j > point_x_min and j <= point_x_max: # TODO: check!
                        for point in point_list_abs:
                            if point.x == j and point.y == i:
                                if temp[row_big-1-i,j].all() == 0: # overwrite it only if it was before black
                                    temp[row_big-1-i,j] = 100 #  100 = grey = occupied; 0 = black = free
                                #if temp[row_big-1-(i-point_y_min),j-point_x_min-1].all() == 0: # overwrite it only if it was before black
                                #    temp[row_big-1-(i-point_y_min),j-point_x_min-1] = 100 #  100 = grey = occupied; 0 = black = free
    cv2.imwrite("map_laser_scan.png", temp) # will be saved in folder $HOME\.ros
    
    # mark the whole area seen from the robot grey no matter if it is occupied or free (for tracking what the robot sees while moving and what remains unseen):
    #mark_laser_scan_area(row_big, col_big, row_start, row_end, col_start, col_end)

def mark_laser_scan_area(row_big, col_big, row_start, row_end, col_start, col_end):
    my_file = Path("map_laser_scan_seen_area.png")
    if not(my_file.is_file()): # the file does not exist
        temp = np.zeros((row_big,col_big)) # (row,col) vs. (row,col,3)
    else:
        temp = cv2.imread("map_laser_scan_seen_area.png")
        for i in range(row_big):
            if i > row_start and i <= row_end:
                for j in range(col_big):
                    if j > col_start and j <= col_end:
                        if temp[row_big-1-i,j].all() == 0: # overwrite it only if it was before black
                            temp[row_big-1-i,j] = 100 #  100 = grey = occupied; 0 = black = free
    cv2.imwrite("map_laser_scan_seen_area.png", temp) # will be saved in folder $HOME\.ros

def callback(data):
    r_pos_x = robot_pos_x # to be sure that they match with the current laser scan
    r_pos_y = robot_pos_y
    r_angle = robot_angle_yaw_grad
    # Read the data from the laser scan (only once):
    global read_laser_scan_info
    if read_laser_scan_info == 1:
        print('Laser scan info:\n')
        print('angle_min:' + str(data.angle_min) + '\n') # start angle of the scan [rad] # -1.57079637051 rad = -90 deg
        print('angle_max:' + str(data.angle_max) + '\n') # end angle of the scan [rad] # 4.69493579865 rad = +270 deg
        print('angle_increment:' + str(data.angle_increment) + '\n') # angular distance between measurements [rad] # 0.0174532923847 rad = 1 deg
        print('time_increment:' + str(data.time_increment) + '\n') # time between measurements [sec] # 0.0
        print('scan_time:' + str(data.scan_time) + '\n') # time between scans [sec] # 0.0
        print('range_min:' + str(data.range_min) + '\n') # minimum range value [m] # 0.0
        print('range_max:' + str(data.range_max) + '\n') # maximum range value [m] # 8.0
        print('ranges:' + str(data.ranges) + '\n') # range data [m] # a huge array of values # len(data.ranges) = 360 # element = nan, when too slose or too far away
        print('intensities:' + str(data.intensities) + '\n') # intensity data # empty
        # Example of ranges:(4.5, 4.500685214996338, 4.502742767333984, 4.506175518035889, 4.510988235473633, 4.517189025878906, 4.524786949157715, 4.533794403076172, 4.544224262237549, 4.556092739105225, 4.569419860839844, 4.584225177764893, 4.600532531738281, 4.618368625640869, 4.637761116027832, 4.658742904663086, 4.681347370147705, 4.705612659454346, 4.731579780578613, 4.759293079376221, 4.78879976272583, 4.820152759552002, 4.8534064292907715, 4.888621807098389, 4.925863265991211, 4.965200424194336, 5.00670862197876, 5.0504679679870605, 5.096565246582031, 5.1450934410095215, 5.196152210235596, 5.249850273132324, 5.306303024291992, 5.365634441375732, 5.427980899810791, 5.493485450744629, 5.562305927276611, 5.634610176086426, 5.7105817794799805, 5.790417671203613, 5.874332904815674, 5.962558269500732, 6.055346965789795, 6.152973651885986, 6.255735874176025, 6.363961219787598, 6.478003978729248, 6.5982561111450195, 6.725144386291504, 6.8591389656066895, 7.000756740570068, 7.150570869445801, 7.309211254119873, 7.477380275726318, 7.655857563018799, 7.8455095291137695, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, 7.796322345733643, 7.664480686187744, 7.651384353637695, 7.72522497177124, 7.037729263305664, 6.9982075691223145, 7.02544641494751, 7.1818389892578125, 7.510401248931885, 7.512060165405273, 7.098311901092529, 7.039268970489502, 7.051368713378906, 7.154397010803223, 7.7110819816589355, 7.605920314788818, 7.601510047912598, 7.688368797302246, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, 4.070480823516846, 3.8512229919433594, 3.7228245735168457, 3.7459053993225098, 3.3208706378936768, 3.176915407180786, 3.045811891555786, 3.032142400741577, 3.056356430053711, 2.7148077487945557, 2.621406316757202, 2.5349643230438232, 2.4547643661499023, 2.3801867961883545, 2.3565993309020996, 2.3819401264190674, 2.408573865890503, 2.4365601539611816, 2.4659643173217773, 2.496857166290283, 2.529313802719116, 3.1837191581726074, 3.114619255065918, 3.0493643283843994, 2.987678289413452, 2.9293136596679688, 2.8740439414978027, 2.8216652870178223, 2.8284263610839844, 2.879112720489502, 2.9325578212738037, 2.988953113555908, 3.0485055446624756, 3.11144757270813, 3.1780309677124023, 2.703127384185791, 2.667165756225586, 2.6329400539398193, 2.6003634929656982, 2.569356679916382, 2.5398452281951904, 2.5117597579956055, 2.5823323726654053, 2.6599998474121094, 2.743344306945801, 2.8329710960388184, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, 7.845513820648193, 7.655858993530273, 7.477382659912109, 7.309212684631348, 7.150572299957275, 7.000757694244385, 6.859140396118164, 6.725146293640137, 6.598257541656494, 6.478005886077881, 6.363961696624756, 6.2557373046875, 6.152975082397461, 6.055347919464111, 5.962559700012207, 5.874334335327148, 5.790417671203613, 5.710582256317139, 5.634611129760742, 5.562306880950928, 5.493487358093262, 5.427980422973633, 5.365634918212891, 5.306303024291992, 5.249851226806641, 5.19615364074707, 5.1450934410095215, 5.096565246582031, 5.050468444824219, 5.006709575653076, 4.965201377868652, 4.925863265991211, 4.888621807098389, 4.8534064292907715, 4.820152759552002, 4.7888007164001465, 4.759293079376221, 4.7315802574157715, 4.705613136291504, 4.681347846984863, 4.658743381500244, 4.637761116027832, 4.618368625640869, 4.6005330085754395, 4.584225654602051, 4.5694193840026855, 4.556093215942383, 4.544224262237549, 4.533794403076172, 4.524787425994873, 4.517189025878906, 4.510988712310791, 4.506175518035889, 4.502742767333984, 4.500685214996338)
        read_laser_scan_info = 0

    # convert laser scan data to a point cloud to get a set of 3D Cartesian (x,y,z) points:
    pc2_msg = lp.projectLaser(data)
    point_generator = pc2.read_points(pc2_msg) # a generator of the individual points (x,y,z,index) (could be accessed in a loop)
    point_list = pc2.read_points_list(pc2_msg) # a list of the individual points (x,y,z,index) # example: Point(x=-0.07854820042848587, y=-4.499999523162842, z=0.0, index=359)
    
    # do sth with the converted data:
    #calculate_avg_x_y_value(point_generator)
    #print(len(point_list)) # at a different timestamp it could always have a different length
    # Important: the (x,y,z) points are relative to the position of the robot
    print(point_list[len(point_list)/2]) # access the information (x,y,z,index) about the middle point of the list # for debugging
    print('robot current absolute pos(x,y): ' + str(robot_pos_x) + ',' + str(robot_pos_y))
    print('robot current angle z: ' + str(robot_angle_yaw_grad))
    # having the absolute robot coordinates translate the relative point cloud data to an absolute data and visualize it on the map:
    # Important: this function is too slow and because of that other functions like visualizing the local costmap can not work correctly!
    #viualize_laser_scan_data(point_list, r_pos_x, r_pos_y, robot_angle_yaw_grad)

    # TODO: preprocessing: save the data (into a rosbag / csv file)
    # TODO: postprocessing: transform the laser scan data to a semantic laser scan data (transform to a 2d bird eye view map and add semantics like 'table1')
    # -> it was done with the local costmap insted!

timer_bool = "yes"

def timeout():
    print("No costmap received for 5 seconds")
    global timer_bool
    timer_bool = "yes"
    pub_timer = rospy.Publisher('costmap_timer_done', String, queue_size=10)
    pub_timer.publish(timer_bool) # "yes"
    # when "yes" is published, the robot should continue moving, so that the topic has again data to publish

timer = threading.Timer(5,timeout) # If 5 seconds elapse, call timeout()
timer.start()

def timer_func():
    # By using a timer you can check when no message are received.
    # If a message is received, you reset the timer (by cancelling and starting a new timer).
    # If no message are received, then you can do what you want in the timeout() function.
    # (https://answers.ros.org/question/334695/how-to-check-whether-the-topic-is-publishing-msg/)
    global timer, timer_bool
    # idea1: publish both "no" (almost the whole time) and "yes" for the 'critical moments'
    # idea2 (used): publish only "yes" when necessary, when the robot should wait so that the whole laser scan data is taken
    if timer_bool == "yes":
        pub_timer = rospy.Publisher("/costmap_timer_done", String, queue_size=10)
        pub_timer.publish(timer_bool)
    timer_bool = "no" # change only after the first "yes" per period
    timer.cancel() # reset the timer
    timer = threading.Timer(5,timeout)
    timer.start()

def callback_local_costmap(map_data):
    timer_func() # Important: this function should be here and not in local_costmap(); otherwise the robot may not move at all (so the imagination_map_global_grey.png should be deleted, the console restared, .. for it to work, and sometimes even this does not help!?); makes a difference of course only for version 2;
    
    # version 1: save the local_costmap by every change
    #local_costmap(map_data)
    
    # version 2: Important: save the local_costmap only every X seconds (for example very 3 seconds) (see move_to_goal.py!)
    # -> but then remove the same check from move_to_goal.py?!!
    # --> ! so dauert es deutlich kuerzer, es muss nur weiterhin das problem mit dem folgenden goal geloest werden!?!
    global temp_time, time_start, img_sec
    # TODO X: the images have inconsistent time stamp, sometimes every 9 seconds, or 6 or .. !?
    img_sec = 3 # 3 -> 5? (because the timer waits also for 5 sec!?)
    temp_time = rospy.get_rostime().secs
    i = 0
    while temp_time >= time_start + i*img_sec:
        if temp_time == time_start + i*img_sec:
            local_costmap(map_data)
        i += 1

def local_costmap(map_data):
    #global map_resolution, x_offset, y_offset, x_max, y_max
    #x_offset = -6
    #y_offset = -6
    #x_max = 521
    #y_max = 666
    map_resolution, x_offset, y_offset, x_max, y_max = get_map_parameters()

    # calculate how much it takes to save the local_costmap as the desired map image
    time_A = rospy.get_rostime()

    # timer_func() # moved to callback_local_costmap() !

    print("Costmap received")
    local_costmap_resolution = map_data.info.resolution # 0.05 # width: 666 [px] * 0.05 (resolution) = 33.3 [m]
    local_costmap_width = map_data.info.width # 60 => 60 [px] * 0.05 (resolution) = 3.0 [m]
    local_costmap_height = map_data.info.height # 60
    robot_map_origin_x = x_offset # -6
    robot_map_origin_y = y_offset # -6
    # robot_pos_x, robot_pos_y come from /odom, not perfect/entirely correct on time!?!
    robot_pos_px_x = int((robot_pos_x - robot_map_origin_x) / local_costmap_resolution)
    robot_pos_px_y = int((robot_pos_y - robot_map_origin_y) / local_costmap_resolution)
    # ! The origin of the map [m, m, rad]. This is the real-world pose of the cell (0,0) in the map. => the bottom left corner of the 60x60 costmap block!
    # => (TODO) the robot should be in the middle of the costmap => the difference between robot_pos_x/y and costmap_origin_x/y should be 60/2=30px
    costmap_origin_x = map_data.info.origin.position.x
    costmap_origin_y = map_data.info.origin.position.y
    costmap_orientation_z = map_data.info.origin.orientation.z # not used, always z = 0 rad
    costmap_origin_px_x = int((costmap_origin_x - robot_map_origin_x) / local_costmap_resolution)
    costmap_origin_px_y = int((costmap_origin_y - robot_map_origin_y) / local_costmap_resolution)
    map_data_array = map_data.data
    map_data_length = len(map_data_array) # 3600
    print('LOCAL COSTMAP: \nlength: ' + str(map_data_length))
    #ground_truth_colored_map = cv2.imread("map_obstacles.png") # with white borders, but definetely not loosing data
    if not(os.path.isfile("./map_ground_truth_semantic.png")):
        error_msg = "\nThere is no ground truth data generated! Launch pedsim_test_gt.launch first.\nRun for example $ roslaunch arena_bringup pedsim_test_gt.launch scenario:=1 gt_extension:=0\n"
        #sys.exit(error_msg)
        raise rospy.ROSInternalException(error_msg)
    ground_truth_colored_map = cv2.imread("map_ground_truth_semantic.png") # ! with colored borders if everyhting works corectly (if a mistake occured, then grey or black)
    # Important: choose the same solution for the borders for both the local costmap data and ground truth data:
    # 'with white borders':
    #    real: ground_truth_colored_map = cv2.imread("map_obstacles.png") -> "map_local_costmap.png" -> "map_local_costmap_part_color.png"
    #   ideal: "map_obstacles_part.png" (cut from "map_obstacles.png")
    # 'with colorful borders':
    #    real: ground_truth_colored_map = cv2.imread("map_ground_truth_semantic.png") -> "map_local_costmap.png" -> "map_local_costmap_part_color.png"
    #   ideal: "map_ground_truth_semantic_part.png" (cut from "map_ground_truth_semantic.png")

    optimization = 1 # TODO X: optimize further, reduce some of the not necessarily generated images in /.ros

    row_big = x_max # height = 521
    col_big = y_max # width = 666

    # Important: for rviz origin is on the bottom left, for an image is always on the top left; bottom left corner is for this map (-6,-6)! => corect the robot's position so that it is always positive
    # TODO: get params like resolution, origin etc. directly from the map yaml file instead of hard-coding them
    block_abs_width_left_rviz = costmap_origin_px_x
    block_abs_width_right_rviz = costmap_origin_px_x + local_costmap_width
    block_abs_height_bottom_rviz = costmap_origin_px_y
    block_abs_height_top_rviz = costmap_origin_px_y + local_costmap_height
    # TODO NEXT: check that the robot is in the middle of the laser scanned costmap (should be the case!)
    
    # save the map as a grey image (black = free; grey = occupied; unknown = white):
    # the local costmap consits of a 60px x 60px block with origin positioned in the bottom left corner
    map_data_array2 = np.array(map_data_array)
    map_reshaped = map_data_array2.reshape(local_costmap_height,local_costmap_width)
    print("costmap: " + str(map_reshaped))
    row,col = map_reshaped.shape
    if optimization == 0:
        temp = np.zeros((row,col)) # (row,col) vs. (row,col,3)
        # the occupancy grid is in row-major order, starting with (0,0); our (0,0) is in the left down corner; for an image it is the upper left corner => mirror the pixels regarding the x axis to be right
        for i in range(row):
            for j in range(col):
                if(map_reshaped[i,j]==-1):
                    temp[row-1-i,j]=255 # unknown = white
                else:
                    if map_reshaped[i,j] == 100: # == 100 or > 90 # filter out the dark grey color -> leave the source (grey=~100) preferably without distortion
                        temp[row-1-i,j]=map_reshaped[i,j] # black = free; grey = occupied
        #cv2.imshow("map_local_costmap_part", temp)
        cv2.imwrite("map_local_costmap_part.png", temp) # will be saved in folder $HOME\.ros # updates every time the robot moves
        #cv2.waitKey(0)
        # TODO: map_local_costmap_part.png is for now not used, it shows the costmap with the overlapping of the imagination for comparison, if not wanted, just cut an image from map_local_costmap.png, as done for the semantic one

    # collect all local costmap data while the robot is moving and update the map image:
    # 1) create a black image with the size of the map image (at the beginning everything is black = 0 = free)
    # 2) from the local costmap 60x60 blocks, calculate the absolute coordinates in the map image
    # 3) update with the robot's movement the black image every time with the local 60x60 part
    # 4) always have the newest image saved (after the robot has moved in the whole space, every pixel should be definted to occupied/free/unknown)

    # Debugging: check again if the corners of the small images are correctly displayed
    # preferably there should be already a black image with this name and with the right dimensions in the $HOME\.ros folder,
    # if not - an image will be generated, but this takes time and in between the robot moves => some local costmap data can be missed and not be visualized
    #img_map_topic = cv2.imread("map_topic.png")
    global map_topic_2darray
    print("map2: " + str(map_topic_2darray)) # map_topic_2darray[0,0] == 100, map_topic_2darray[0,0] == 101
    img_global_costmap = cv2.imread("map_global_costmap.png") # img_global_costmap[0,0] == [100 100 100]
    my_file = Path("map_local_costmap.png")
    if not(my_file.is_file()): # the file does not exist
        temp_img = np.zeros((row_big,col_big,3)) # size of the big map image # (row_big,col_big,3), because the black color (0,0,0) is needed!
        temp_img_grey = np.zeros((row_big,col_big,3))
    else:
        temp_img = cv2.imread("map_local_costmap.png")
        temp_img_grey = cv2.imread("map_local_costmap_grey.png") # should definetely include all laser scan points (in grey)
        for i in range(row_big):
            if i > block_abs_height_bottom_rviz and i <= block_abs_height_top_rviz:
                for j in range(col_big):
                    if j > block_abs_width_left_rviz and j <= block_abs_width_right_rviz:
                        if(map_reshaped[i-block_abs_height_bottom_rviz-1,j-block_abs_width_left_rviz-1]==-1):
                            temp_img[row_big-1-i,j] = 255 # unknown = white
                            temp_img_grey[row_big-1-i,j] = 255 # unknown = white
                        else:
                            if temp_img_grey[row_big-1-i,j].all() == 0: # overwrite it only if it was before black (init status 'free'); if it was grey, leave it grey and do not overwrite it with black
                                if map_reshaped[i-block_abs_height_bottom_rviz-1,j-block_abs_width_left_rviz-1] == 100: # == 100 or > 90 # filter out the dark grey color -> leave the source (grey=~100) preferably without distortion
                                    temp_img_grey[row_big-1-i,j] = map_reshaped[i-block_abs_height_bottom_rviz-1,j-block_abs_width_left_rviz-1] # black = free; grey = occupied
                            if temp_img[row_big-1-i,j].all() == 0: # overwrite it only if it was before black (init status 'free'); if it was grey, leave it grey and do not overwrite it with black
                                if map_reshaped[i-block_abs_height_bottom_rviz-1,j-block_abs_width_left_rviz-1] == 100: # == 100 or > 90 # filter out the dark grey color -> leave the source (grey=~100) preferably without distortion
                                    # add semantics => do not color it always grey, but take the color from the ground truth map based on the global position
                                    color_r1 = ground_truth_colored_map[row_big-1-i, j, 2] # everything visualized in opencv is in form BGR and not RGB!
                                    color_g1 = ground_truth_colored_map[row_big-1-i, j, 1]
                                    color_b1 = ground_truth_colored_map[row_big-1-i, j, 0]
                                    temp_img[row_big-1-i,j] = (color_b1,color_g1,color_r1) # everything visualized in opencv is in form BGR and not RGB!
                                    #if optimization == 0:
                                    # Idea: get the color from the neighbours: row_big-2-i vs. row_big-1-i & j vs. j+1 -> if the color is black, check the color of the upper-left (3/8) of the neighbours:
                                    color_r2 = ground_truth_colored_map[row_big-1-i, j+1, 2]
                                    color_g2 = ground_truth_colored_map[row_big-1-i, j+1, 1]
                                    color_b2 = ground_truth_colored_map[row_big-1-i, j+1, 0]
                                    color_r3 = ground_truth_colored_map[row_big-2-i, j, 2]
                                    color_g3 = ground_truth_colored_map[row_big-2-i, j, 1]
                                    color_b3 = ground_truth_colored_map[row_big-2-i, j, 0]
                                    color_r4 = ground_truth_colored_map[row_big-2-i, j+1, 2]
                                    color_g4 = ground_truth_colored_map[row_big-2-i, j+1, 1]
                                    color_b4 = ground_truth_colored_map[row_big-2-i, j+1, 0]
                                    # TODO Debugging: check maybe also the rest of the neighbours (check in a radius bigger then 1 px!?)
                                    # do it like on another place, get the color that occurs the most and is not black, white or grey
                                    color_r5 = ground_truth_colored_map[row_big-1-i, j-1, 2]
                                    color_g5 = ground_truth_colored_map[row_big-1-i, j-1, 1]
                                    color_b5 = ground_truth_colored_map[row_big-1-i, j-1, 0]
                                    color_r6 = ground_truth_colored_map[row_big-2-i, j-1, 2]
                                    color_g6 = ground_truth_colored_map[row_big-2-i, j-1, 1]
                                    color_b6 = ground_truth_colored_map[row_big-2-i, j-1, 0]
                                    color_r7 = ground_truth_colored_map[row_big-2-i, j-1, 2]
                                    color_g7 = ground_truth_colored_map[row_big-i, j-1, 1]
                                    color_b7 = ground_truth_colored_map[row_big-i, j-1, 0]
                                    color_r8 = ground_truth_colored_map[row_big-i, j, 2]
                                    color_g8 = ground_truth_colored_map[row_big-i, j, 1]
                                    color_b8 = ground_truth_colored_map[row_big-i, j, 0]
                                    color_r9 = ground_truth_colored_map[row_big-i, j+1, 2]
                                    color_g9 = ground_truth_colored_map[row_big-i, j+1, 1]
                                    color_b9 = ground_truth_colored_map[row_big-i, j+1, 0]
                                    # laser scan data, that couldn't have been colored properly because of the ground truth data:
                                    if color_r1 == 0 and color_g1 == 0 and color_b1 == 0: # if black, take a neighbour color with the hope of not being black
                                        temp_img[row_big-1-i,j] = (color_b2,color_g2,color_r2)
                                        if color_r2 == 0 and color_g2 == 0 and color_b2 == 0:
                                            temp_img[row_big-1-i,j] = (color_b3,color_g3,color_r3)
                                            if color_r3 == 0 and color_g3 == 0 and color_b3 == 0:
                                                temp_img[row_big-1-i,j] = (color_b4,color_g4,color_r4)
                                                if color_r4 == 0 and color_g4 == 0 and color_b4 == 0:
                                                    temp_img[row_big-1-i,j] = (color_b5,color_g5,color_r5)
                                                    if color_r5 == 0 and color_g5 == 0 and color_b5 == 0:
                                                        temp_img[row_big-1-i,j] = (color_b6,color_g6,color_r6)
                                                        if color_r6 == 0 and color_g6 == 0 and color_b6 == 0:
                                                            temp_img[row_big-1-i,j] = (color_b7,color_g7,color_r7)
                                                            if color_r7 == 0 and color_g7 == 0 and color_b7 == 0:
                                                                temp_img[row_big-1-i,j] = (color_b8,color_g8,color_r8)
                                                                if color_r8 == 0 and color_g8 == 0 and color_b8 == 0:
                                                                    temp_img[row_big-1-i,j] = (color_b9,color_g9,color_r9)
                                                                    if color_r9 == 0 and color_g9 == 0 and color_b9 == 0:
                                                                        temp_img[row_big-1-i,j] = (100,100,100) # if still black, color it grey to not loose a laser scan information only because of the ground truth
                            ## Important: the local_costmap will always have only the values 0, 100 or -1, so there is no way to make difference between the obstacles (which are coming from the changed map or from the laser scan)!?
                            ## Important idea (TODO): delete (make free again) the imagination area from the local costmap image (still part of the topic so that the robot does not move over, but not part of the pair images - inputs to the imagination model!)
                            ## mark in black/grey/color the imagination area (doesn't help, because it marks over the laser scans):
                            #if map_topic_2darray[i,j] == 101:
                            #    temp_img_grey[row_big-1-i,j] = (0,0,0)
                            #    temp_img[row_big-1-i,j] = (0,255,0)
                            ## mark in black/grey/color the area that is not grey in the global_costmap (there are only the laser scans grey, no imagination there => it works!) (do it both for the grey and colored map: temp_img_grey & temp_img)
                            ## use the green color (0,255,0) for debugging, to see what has been colored, then change to black (0,0,0) = free:
                            if len(map_topic_2darray) != 0:
                                if map_topic_2darray[i,j] == 101: # optimization: check only there where an imagination is
                                    if not(img_global_costmap[row_big-1-i,j][0] == 100 and img_global_costmap[row_big-1-i,j][1] == 100 and img_global_costmap[row_big-1-i,j][2] == 100):
                                        temp_img_grey[row_big-1-i,j] = (0,0,0)
                                        temp_img[row_big-1-i,j] = (0,0,0)
    cv2.imwrite("map_local_costmap.png", temp_img) # will be saved in folder $HOME\.ros
    cv2.imwrite("map_local_costmap_grey.png", temp_img_grey)

    # prepare the ground truth data for comparing it with the local (60x60 block) data from the costmap
    if optimization == 0:
        # 1) cut from the ground truth map (obstacles map) exactly the same 60x60 block
        ground_truth_map = cv2.imread("map_obstacles.png")
        ground_truth_map_part = ground_truth_map[row_big-1-block_abs_height_top_rviz+1:row_big-1-block_abs_height_bottom_rviz+1, block_abs_width_left_rviz+1:block_abs_width_right_rviz+1]
        cv2.imwrite("map_obstacles_part.png", ground_truth_map_part)
    # 1) alternative: cut from the ground truth map
    if not(os.path.isfile("./map_ground_truth_semantic.png")):
        error_msg = "\nThere is no ground truth data generated! Launch pedsim_test_gt.launch first.\nRun for example $ roslaunch arena_bringup pedsim_test_gt.launch scenario:=1 gt_extension:=0\n"
        #sys.exit(error_msg)
        raise rospy.ROSInternalException(error_msg)
    ground_truth_map_2 = cv2.imread("map_ground_truth_semantic.png")
    ground_truth_map_60x60 = ground_truth_map_2[row_big-1-block_abs_height_top_rviz+1:row_big-1-block_abs_height_bottom_rviz+1, block_abs_width_left_rviz+1:block_abs_width_right_rviz+1]
    if optimization == 0: cv2.imwrite("map_ground_truth_semantic_part.png", ground_truth_map_60x60)
    # 2) cut from the big colorful local cost map exactly the same 60x60 block
    local_costmap_60x60 = temp_img[row_big-1-block_abs_height_top_rviz+1:row_big-1-block_abs_height_bottom_rviz+1, block_abs_width_left_rviz+1:block_abs_width_right_rviz+1]
    if optimization == 0: cv2.imwrite("map_local_costmap_part_color.png", local_costmap_60x60)
    # 3) compare the local costmap part image with the ground truth part image: local_costmap_60x60 (real) vs. ground_truth_map_part (ideal)
    # -> multiple examples of such pairs are the input of the neural network for training the imagination unit

    # TODO X: resize the data from 60x60 to 80x80/100x100/160x160/...
    imagination_size = int(rospy.get_param('~imagination_size')) # 60/80/100/160
    #if imagination_size != 60:
    range_old = 60
    range_new = imagination_size

    # resize the observation 60x60 img to an 80x80/100x100/... img with white color (unknown, id = -1?) from all sides (robot should be still in the middle), so that it could be used with a 80x80/100x100/... ground truth image
    #local_costmap_resized = np.zeros((range_new,range_new,3)) # init with [0,0,0]=black
    local_costmap_resized = np.full((range_new,range_new,3), 255) # init with [255,255,255]=white # optimization!
    step = int((range_new - range_old)/2) # (80-60)/2=10 # [px] -> 10 [px] * 0.05 (resolution) = 0.5 [m]
    for i in range(range_new):
        for j in range(range_new):
            if not((i < step or i >= (range_old + step)) or (j < step or j >= (range_old + step))): # border from all sides: 0-9 & 70-79
                local_costmap_resized[i,j] = local_costmap_60x60[i-step,j-step] # take the color only in the middle, the inside of the borders
                #print(str(i) + ' ' + str(j) + ' ' + str(i-step) + ' ' + str(j-step)) # debugging

    # cut 80x80/100x100/160x160 from the ground truth map & make sure that the robot is in the middle of the block 
    # for example: 80x80 => -10px left, +10px right, +10px top, -10px bottom
    ground_truth_map_resized = image_cut(ground_truth_map_2, step, row_big, block_abs_width_left_rviz, block_abs_width_right_rviz, block_abs_height_bottom_rviz, block_abs_height_top_rviz)
    if optimization == 0: cv2.imwrite("map_ground_truth_semantic_part_resized.png", ground_truth_map_resized)
    
    # Important: to get the local costmap, also the teleoperation could be used to drive the robot manually around (could be faster then running a script to random go through the room)

    ## publish the important images (I, II and III) to topics:

    # I - map_local_costmap_part_color.png
    ## publish Image - converting OpenCV images to ROS image messages
    #publish_image('costmap_temp', local_costmap_60x60) # works, but cvbridge does not work in rosnav python environment
    ## try publishing an array instead of an image (create an user-defined ros message: http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)
    #publish_listintlist('costmap_temp', local_costmap_60x60) # does not work # with 'ListIntList' or 'ListListIntList' => NotImplementedError: multi-dimensional sub-views are not implemented
    ## publish IntList - convert OpenCV images to user-defined array messages
    #publish_intlist('costmap_temp', local_costmap_60x60) # works # (60,60,3) -> (10800,1,1)
    ## publish OccupancyGrid
    grid_costmap_60x60 = publish_occupancygrid('costmap_temp', local_costmap_resized, map_data)

    ## TODO: the whole global costmap, ("map_local_costmap.png", temp_img) vs. ("map_local_costmap_grey.png", temp_img_grey)
    #publish_intlist('costmap_global_temp', temp_img_grey) # works # (60,60,3) -> (10800,1,1)

    # II - map_ground_truth_semantic_part.png
    ## publish Image - converting OpenCV images to ROS image messages
    #publish_image('ground_truth_map_temp', ground_truth_map_60x60) # works, but cvbridge does not work in rosnav python environment
    ## publish IntList - convert OpenCV images to user-defined array messages
    #publish_intlist('ground_truth_map_temp', ground_truth_map_60x60) # works # (60,60,3) -> (10800,1,1)
    ## publish OccupancyGrid
    grid_ground_truth_60x60 = publish_occupancygrid('ground_truth_map_temp', ground_truth_map_resized, map_data)

    ## publish the ground truth and costmap as a pair -> a list of OccupancyGrids
    publish_pairoccupancygrid('pair_temp', grid_costmap_60x60, grid_ground_truth_60x60)

    if optimization == 0:
        # III - map_obstacles_part.png
        ## publish Image - converting OpenCV images to ROS image messages
        #publish_image('obstacles_map_temp', ground_truth_map_part) # works, but cvbridge does not work in rosnav python environment
        ## publish IntList - convert OpenCV images to user-defined array messages
        #publish_intlist('obstacles_map_temp', ground_truth_map_part) # works # (60,60,3) -> (10800,1,1)
        ## publish OccupancyGrid
        grid_obstacles = publish_occupancygrid('obstacles_map_temp', ground_truth_map_part, map_data)

    time_B = rospy.get_rostime()
    print('laser scan delay = ' + str(time_B - time_A) + ' ns') # 1760000000 ns = 1.76 sec # no optimization: 1.7-2.0 sec # optimization: 0.6-1.15 sec -> 0.34-0.93 sec

def publish_image(publisher_name, temp_img_part):
    ## map_local_costmap_part_color.png
    ## publish Image - converting OpenCV images to ROS image messages
    pub = rospy.Publisher(publisher_name, Image, queue_size=10)
    bridge = CvBridge()
    image_message = bridge.cv2_to_imgmsg(temp_img_part, encoding="passthrough")
    pub.publish(image_message)

def publish_listintlist(publisher_name, temp_img_part): # does not work
    ## try publishing an array instead of an image (create an user-defined ros message: http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)
    pub_costmap = rospy.Publisher(publisher_name, ListListIntList, queue_size=10)
    image_message = asarray([temp_img_part]) # example shape: (200, 400, 3) # [[[ 0  0  0] [ 0  0  0]] ..., [[ 0  0  0] [ 0  0  0]]]
    pub_costmap.publish(image_message)
    ## with types 'ListIntList' or 'ListListIntList' => NotImplementedError: multi-dimensional sub-views are not implemented

def publish_intlist(publisher_name, temp_img_part):
    ## try publishing an array instead of an image (create an user-defined ros message: http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv)
    ## publish IntList - convert OpenCV images to user-defined array messages
    # convert from (X,Y,3) to (A,B,1) to (Z,1,1) => from (60,60,3) to (10800,1,1)
    # convert from np.array to list (with or without commas between the elements)
    pub = rospy.Publisher(publisher_name, IntList, queue_size=10)
    list_message = asarray(temp_img_part) # shape [60, 60, 3]
    list_message_reshaped = list_message.reshape(-1) # list()
    list_message_int = []
    for i in list_message_reshaped:
        # TODO: int (=np.int32 or np.int16) -> np.int8 changes the color of the image!!
        list_message_int.append(int(i))
    #print(list_message_int) # [0, 0, 0, ... 0, 0, 0]
    pub.publish(list_message_int)

def publish_occupancygrid(publisher_name, map, map_data):
    ## publish OccupancyGrid
    pub = rospy.Publisher(publisher_name, OccupancyGrid, queue_size=10)
    grid = OccupancyGrid()
    grid.header.seq = 0
    grid.header.stamp = rospy.Time.now()
    grid.header.frame_id = ""
    grid.info = map_data.info
    # TODO: grid.data must be int8
    # TODO: the array int8[] includes IDs like 0=free, -1=unknown, 100=occupied, 1 to 10 for colored occupied
    # => take the grey image and from [0,0,0] => 0; [100,100,100] => 100
    list_message = asarray(map)
    #row,col,color = list_message.shape # (60,60,3) -> (60,60) # TODO: sometimes at the very beggining an error occures that the shape is (60,60) instead of (60,60,3)
    row = list_message.shape[0]
    col = list_message.shape[1]
    id_type_color_ar = type_color_reference()
    #array_new = get_id_from_color(list_message) # TODO NEXT: make it colorful!?
    array_new = np.zeros((row,col)) # black = free # (row,col) vs. (row,col,3)
    # the occupancy grid is in row-major order, starting with (0,0); our (0,0) is in the left down corner; for an image it is the upper left corner => mirror the pixels regarding the x axis to be right
    # TODO: array order!?
    for i in range(row):
        for j in range(col):
            if(list_message[row-1-i,j][0] != 0 or list_message[row-1-i,j][1] != 0 or list_message[row-1-i,j][2] != 0):
            ##if(list_message[i,j][0] != 0 or list_message[i,j][1] != 0 or list_message[i,j][2] != 0):
                ##array_new[row-1-i,j] = 100 # color 1 to 10 /grey 100 = occupied # wrong order!
                #array_new[i,j] = 100 # color 1 to 10 /grey 100 = occupied # correct order!
                array_new[i,j] = 0 # make the area per default better black=free then grey, since we want to filter out the imagination!
                if list_message[row-1-i,j][2] == 255 and list_message[row-1-i,j][1] == 255 and list_message[row-1-i,j][0] == 255:
                    array_new[i,j] = -1 # TODO X
                if list_message[row-1-i,j][2] == 100 and list_message[row-1-i,j][1] == 100 and list_message[row-1-i,j][0] == 100:
                    array_new[i,j] = 100 # TODO X # 1 or 100 (should be the same as 'GREY DEFAULT')
                for elem in id_type_color_ar: # TODO: make it colorful
                    #print('RED from RGB: ' + str(int(elem['color'][0]*255)) + ' ' + str(BGR_color[0])) # for debugging
                    # Important: RGB vs. BGR!
                    if int(elem['color'][0]*255) == list_message[row-1-i,j][2] and int(elem['color'][1]*255) == list_message[row-1-i,j][1] and int(elem['color'][2]*255) == list_message[row-1-i,j][0]:
                        array_new[i,j] = elem['id']
                        #array_new[i,j] = 100 # 1 or 100 (should be the same as 'GREY DEFAULT') # one layer for now! already done in CustomDatasetSingle() (TODO)
                        break
    array_new_reshaped = array_new.reshape(-1)
    array_new_int = []
    for i in array_new_reshaped:
        array_new_int.append(int(i))
    grid.data = array_new_int
    pub.publish(grid)
    #print('***')
    #print(array_new.shape) # (60, 60)
    #print(array_new_reshaped.shape) # (3600,)
    #print(len(array_new_int)) # 3600
    #print(grid.data) # [0 0 0 ... 0 0 0]; [[0. 0. 0. ... 0. 0. 0.] ... [0. 0. 0. ... 0. 0. 0.]]
    #print('***')
    return grid

def get_id_from_color(img_costmap_color):
    id_type_color_ar = type_color_reference()
    row = img_costmap_color.shape[0]
    col = img_costmap_color.shape[1]
    img_costmap_id = np.zeros((row,col)) # (row,col) vs. (row,col,3)
    for i in range(row):
        for j in range(col):
            #BGR_color = [img_costmap_color[row-1-i, j, 0], img_costmap_color[row-1-i, j, 1], img_costmap_color[row-1-i, j, 2]]
            id_temp = 0 # per default 0 = free = no obstacles
            if 100 == img_costmap_color[row-1-i,j][2] and 100 == img_costmap_color[row-1-i,j][1] and 100 == img_costmap_color[row-1-i,j][0]:
                id_temp = 100 # TODO: 100 or 1 .. (should be the same as 'GREY DEFAULT')
            # map here the color to the id:
            for elem in id_type_color_ar:
                #print('RED from RGB: ' + str(int(elem['color'][0]*255)) + ' ' + str(BGR_color[0])) # for debugging
                # Important: RGB vs. BGR!
                if int(elem['color'][0]*255) == img_costmap_color[row-1-i,j][2] and int(elem['color'][1]*255) == img_costmap_color[row-1-i,j][1] and int(elem['color'][2]*255) == img_costmap_color[row-1-i,j][0]:
                    #id_temp = elem['id']
                    id_temp = 100 # 1 or 100 (should be the same as 'GREY DEFAULT') # one layer for now! already done in CustomDatasetSingle() (TODO)
                    break
            img_costmap_id[i,j] = id_temp
    return img_costmap_id

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

def publish_pairoccupancygrid(publisher_name, grid1, grid2):
    ## publish the ground truth and costmap as a pair -> a list of OccupancyGrids
    pub_pair = rospy.Publisher(publisher_name, ListOccupancyGrid, queue_size=10)
    array_pair = [grid1, grid2]
    pub_pair.publish(array_pair)

def image_cut(ground_truth_map_2, border, row_big, block_abs_width_left_rviz, block_abs_width_right_rviz, block_abs_height_bottom_rviz, block_abs_height_top_rviz):
    block_abs_width_left_rviz_PXxPX = block_abs_width_left_rviz - border
    block_abs_width_right_rviz_PXxPX = block_abs_width_right_rviz + border
    block_abs_height_bottom_rviz_PXxPX = block_abs_height_bottom_rviz - border
    block_abs_height_top_rviz_PXxPX = block_abs_height_top_rviz + border
    ground_truth_map_PXxPX =  ground_truth_map_2[row_big-1-block_abs_height_top_rviz_PXxPX+1:row_big-1-block_abs_height_bottom_rviz_PXxPX+1, block_abs_width_left_rviz_PXxPX+1:block_abs_width_right_rviz_PXxPX+1]
    return ground_truth_map_PXxPX

def callback_map(map_data):
    #global map_resolution, x_offset, y_offset, x_max, y_max
    #x_max = 521
    #y_max = 666
    map_resolution, x_offset, y_offset, x_max, y_max = get_map_parameters()
    
    # TODO: wait for the obstacles to be spawned!?
    #print(map_data) # consists of a header, metadata info and a data array, where 0 = free, 100 = occupied, -1 = unknown # whiter pixels are free, blacker pixels are occupied, and pixels in between are unknown
    map_data_array = asarray([map_data.data])
    savetxt('map_data.csv', map_data_array, delimiter=',') # will be saved in folder $HOME\.ros
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
    print("MAP - FREE: " + str(free_amount) + ", UNKNOWN: " + str(unknown_amount) + ", OCCUPIED: " + str(ocupied_amount)) # FREE: 278156, UNKNOWN: 2134, OCCUPIED: 66696

    # save the map as a grey image (black = free; grey = occupied; unknown = white):
    map_data_array2 = np.array(map_data.data) # array of size 346986
    map_reshaped = map_data_array2.reshape((x_max,y_max))
    global map_topic_2darray
    map_topic_2darray = map_reshaped
    print("map: " + str(map_reshaped))
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

def callback_global_costmap(map_data): # TODO: it does not update itself when the robots is moving, even though it does in rviz!?
    #global map_resolution, x_offset, y_offset, x_max, y_max
    #x_max = 521
    #y_max = 666
    map_resolution, x_offset, y_offset, x_max, y_max = get_map_parameters()
    
    print('GLOBAL COSTMAP: ' + str(len(map_data.data))) # 346986 (the same length as the one from /map), but different info: it updates globally with the info from the obstacles while the robot is moving)
    #print(map_data) # consists of a header, metadata info and a data array, where 0 = free, 100 = occupied, -1 = unknown # whiter pixels are free, blacker pixels are occupied, and pixels in between are unknown
    map_data_array = asarray([map_data.data])
    savetxt('global_costmap_data.csv', map_data_array, delimiter=',') # will be saved in folder $HOME\.ros
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
    map_reshaped = map_data_array2.reshape((x_max, y_max))
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

def callback_odom(odom_data):
    #print('(pos_x, pos_y, angle_z) = (' + str(odom_data.pose.pose.position.x) + ', ' + str(odom_data.pose.pose.position.y) + ', ' + str(odom_data.pose.pose.orientation.z) + ')')
    global robot_pos_x
    global robot_pos_y
    global robot_angle_yaw_grad

    robot_pos_x = odom_data.pose.pose.position.x
    robot_pos_y = odom_data.pose.pose.position.y

    q_x = odom_data.pose.pose.orientation.x
    q_y = odom_data.pose.pose.orientation.y
    q_z = odom_data.pose.pose.orientation.z
    q_w = odom_data.pose.pose.orientation.w

    # transformation from geometry_msgs/Pose2d to geometry_msgs/Pose (in 3d)
    # (from quaternions to Euler angles, where only yaw (a z-axis rotation) is interessting)
    siny_cosp = 2 * (q_w * q_z + q_x * q_y)
    cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
    robot_angle_yaw = math.atan2(siny_cosp, cosy_cosp)
    robot_angle_yaw_grad = math.degrees(robot_angle_yaw) # the same as the given 'theta' in the pose2d form in pedsim_test.py

def callback_global_costmap_updates(map_data):
    #print(map_data.data)
    #print('GLOBAL COSTMAP UPDATE: ' + str(len(map_data.data))) # 23220/29068/.. (the length changes; not the same length as the one from /map 346986)
    # TODO: the images are always from a different size; probably locally placed; the amount of occupied and free pixels changes over time

    map_data_array = asarray([map_data.data])
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
    map_data_array2 = np.array(map_data.data) # array of different length
    map_reshaped = map_data_array2.reshape((map_data.width,map_data.height))
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
    cv2.imwrite("map_global_costmap_updates_data.png", temp) # will be saved in folder $HOME\.ros

def callback_mapping(data):
    print(data)

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

def laser_scan_data_listener():
    rospy.init_node('scan_values')

    #global map_resolution, x_offset, y_offset, x_max, y_max
    #x_max = 521
    #y_max = 666
    map_resolution, x_offset, y_offset, x_max, y_max = get_map_parameters()

    # init the global images of the local costmap as black images, so that every roslaunch rewrites them
    temp_black_img = np.zeros((x_max,y_max,3)) # (row,col) vs. (row,col,3) # (521,666) vs. (521,666,3)
    cv2.imwrite("map_local_costmap.png", temp_black_img)
    cv2.imwrite("map_local_costmap_grey.png", temp_black_img)

    #time.sleep(2) # wait for the obstacles to be spawned

    global time_start, temp_time
    time_start = rospy.get_rostime().secs # get only the seconds (it's enough?), then the number do not have to be converted to integer while comparing the times
    temp_time = rospy.get_rostime().secs

    # read the laser scan data and save also the absolute and relative position of the robot the whole time,
    # to be able to match it with the laser scan data; save also the info from the map to know where the obstacles are
    #rospy.Subscriber("/scan", LaserScan, callback) # queue_size=1
    rospy.Subscriber('/map', OccupancyGrid, callback_map) # /move_base/global_costmap/costmap similar to /map # TODO NEXT: the map do not update? the newly spawned obstacles are not visible!
    rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, callback_local_costmap)
    rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, callback_global_costmap) # TODO NEXT: the map do not update although it does in rviz!? the newly spawned obstacles are not visible!
    #rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate, callback_global_costmap_updates) # TODO: an unvizualisable topic in rviz
    #rospy.Subscriber('/slam_gmapping/entropy ', Float64, callback_mapping) # TODO?
    rospy.Subscriber('/odom', Odometry, callback_odom) # /odom returns position and velocity of the robot
    
    rospy.spin()

if __name__ == '__main__':
    laser_scan_data_listener()
