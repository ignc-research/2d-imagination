#!/usr/bin/env python
import rospy
import os
import math
import time
import rospkg
import cv2
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from numpy import asarray, savetxt
from collections import namedtuple
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg
from visualization_msgs.msg import MarkerArray

lp = lg.LaserProjection()
read_laser_scan_info = 1
read_markers_info = 1
amount_obstacles = 0

robot_pos_x = 0.0
robot_pos_y = 0.0
robot_angle_z = 0.0

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

def viualize_laser_scan_data(point_list):
    point_list_abs = []
    resolution = 0.05 # the resolution of the currently used map
    origin_x_y = -6 # the origin of the currently used map is (-6,-6)
    AbsPoint = namedtuple("AbsPoint", "x y")
    for point in point_list:
        x_abs = point.x + robot_pos_x
        y_abs = point.y + robot_pos_y
        x_px = int((x_abs - origin_x_y) / resolution)
        y_px = int((y_abs - origin_x_y) / resolution)
        point_list_abs.append(AbsPoint(x_px, y_px)) # ({'x': x_px, 'y': y_px})
    print(point_list_abs[len(point_list_abs)/2]) # for debugging
    point_x_max = 0
    point_y_max = 0
    point_x_min = 1000
    point_y_min = 1000
    for point in point_list_abs:
        if point.x > point_x_max: point_x_max = point.x
        if point.y > point_y_max: point_y_max = point.y
        if point.x < point_x_min: point_x_min = point.x
        if point.y < point_y_min: point_y_min = point.y
    row_end = point_y_max # find the biggest y
    col_end = point_x_max # find the biggest x
    row_start = point_y_min # find the smallest y
    col_start = point_x_min # find the smallest y
    row = row_end - row_start
    col = col_end - col_start

    rospack = rospkg.RosPack()
    relative_img_path = os.path.join(rospack.get_path("simulator_setup"), "maps", "map_empty", "map_small.png")
    used_map_image = cv2.imread(relative_img_path) # get the size of the used map image: width x height 666 x 521
    row_big,col_big,val = used_map_image.shape

    # for the map and the robot the origin (0,0) is in the left down corner; for an image it is always the upper left corner => mirror the pixels regarding the x axis to be right
    temp_small = np.zeros((row,col))
    for i in range(row_start, row_end+1):
        for j in range(col_start, col_end+1):
            for point in point_list_abs:
                if point.x == j and point.y == i:
                    temp_small[row-1-(i-row_start),j-col_start-1] = 100 #  100 = grey = occupied; 0 = black = free
    cv2.imwrite("map_laser_scan_part.png", temp_small) # will be saved in folder $HOME\.ros

    # TODO NEXT - the visualization is not correct yet:
    # - maybe here is the angle important (not like with the local costmap); check the border values!
    # - the point cloud coordinates are relative to the robot coordinates? where is x and where y axis?
    # - the calculations and iteration are happening maybe just too slowly, the calculated min-max borders to not match anymore with the current laser scan area etc.!?
    # - sometimes 'row_start' and 'col_start' have a negative value!?

    # preferably there should be already a black image with this name and with the right dimensions in the $HOME\.ros folder,
    # if not - an image will be generated, but this takes time and in between the robot moves => some laser scan data can be missed and not be visualized
    my_file = Path("map_laser_scan.png")
    if not(my_file.is_file()): # if the file does not exist
        temp = np.zeros((row_big,col_big))
    else:
        temp = cv2.imread("map_laser_scan.png")
        for i in range(row_big): # row_big - 0:521-1
            if i > row_start and i <= row_end:
                for j in range(col_big): # col_big - 0:666-1
                    if j > col_start and j <= col_end:
                        for point in point_list_abs:
                            if point.x == j and point.y == i:
                                if temp[row_big-1-i,j].all() == 0: # overwrite it only if it was before black
                                    temp[row_big-1-i,j] = 100 #  100 = grey = occupied; 0 = black = free
    cv2.imwrite("map_laser_scan.png", temp) # will be saved in folder $HOME\.ros
    
    # mark the whole area seen from the robot grey no matter if it is occupied or free (for tracking what the robot sees while moving and what remains unseen):
    #mark_laser_scan_area(row_big, col_big, row_start, row_end, col_start, col_end)

def mark_laser_scan_area(row_big, col_big, row_start, row_end, col_start, col_end):
    my_file = Path("map_laser_scan_seen_area.png")
    if not(my_file.is_file()): # the file does not exist
        temp = np.zeros((row_big,col_big))
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
    print('robot current angle z: ' + str(robot_angle_z))
    # having the absolute robot coordinates translate the relative point cloud data to an absolute data and visualize it on the map:
    # Important: this function is too slow and because of that other functions like visualizing the local costmap can not work correctly!
    #viualize_laser_scan_data(point_list)

    # TODO: preprocessing: save the data (into a rosbag / csv file)
    # TODO: postprocessing: transform the laser scan data to a semantic laser scan data (transform to a 2d bird eye view map and add semantics like 'table1')

def callback_map(map_data):
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
    rospack = rospkg.RosPack()
    map_data_array2 = np.array(map_data.data) # array of size 346986
    relative_img_path = os.path.join(rospack.get_path("simulator_setup"), "maps", "map_empty", "map_small.png")
    used_map_image = cv2.imread(relative_img_path) # get the size of the used map image: width x height 666 x 521
    map_reshaped = map_data_array2.reshape((used_map_image.shape[0],used_map_image.shape[1]))
    print(map_reshaped)
    row,col = map_reshaped.shape
    temp = np.zeros((row,col))
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

    # TODO NEXT: a ground truth map with 'occupied', 'not occupied' or 'unknown' is needed
    # the obstacles for the ground truth map should be actually from the biggest layer (layer2), since layer2 (and layer3) should be known after the imagination with layer1
    # -> but the /map topic does not recognize the spawn tables and chairs as an occupied area!?
    # -> the black areas on the map image "map_small.png" are considered as obstacles
    # idea: get the coordinates of the obstacles from the topic /flatland_markers (instead of: params from the obstacle yaml files and the poses from pedsim_test.py) and visualize them on the image map with opencv (cv2.rectangle()) - the /map topic itself could not be updated, but the image with the obstacles could be still internal used

def callback_flatland_markers(markers_data):
    #time.sleep(5) # wait for the obstacles to be spawned!?
    rospack = rospkg.RosPack()
    img_path = os.path.join(rospack.get_path("simulator_setup"), "maps", "map_empty", "map_small.png")
    used_map_image = cv2.imread(img_path)
    row_big,col_big,val = used_map_image.shape
    #print('MARKERS DATA: ' + str(markers_data))
    global read_markers_info
    global amount_obstacles
    amount_obstacles = len(markers_data.markers)
    if amount_obstacles == 142 and read_markers_info == 1: # TODO: it gets time to get all obstacles
        read_markers_info = 0
        counter_spheres = 0
        counter_polygons = 0
        counter_others = 0
        counter_lines = 0
        origin_x = -6
        origin_y = -6
        resolution = 0.05
        obstacle_markers = []
        ObstacleMarker = namedtuple("ObstacleMarker", "x y markers") # a struct per obstacle (center_x, center_y, markers_array)

        # collect all parts that belog to the same obstacle - if center_x and center_y are the same, they are part of the same obstacle (important for later to rotate the whole obstacle at once)
        for marker in markers_data.markers:
            center_x = marker.pose.position.x
            center_y = marker.pose.position.y
            i = 0
            new_center = 1
            for m in obstacle_markers:
                if round(m.x, 2) == round(center_x, 2) and round(m.y, 2) == round(center_y, 2): # round it until the second digit after the comma
                    obstacle_markers[i].markers.append(marker)
                    new_center = 0
                i += 1
            if new_center == 1:
                if not(int(center_x) >= -2 and int(center_x) <= 2 and int(center_y) >= -2 and int(center_y) <= 2): # TODO: try to get the robot out (it should be at (0,0) but maybe it moved)
                    if not(center_x == -6 and center_y == -6): # for the type = line in with center the origin!? (TODO)
                        part_markers = [marker]
                        obstacle_markers.append(ObstacleMarker(center_x, center_y, part_markers)) # ({'x': center_x, 'y': center_y, 'markers': part_markers})
        print('AMOUNT OF OBSTACLES: ' + str(len(obstacle_markers))) # 26
        #for m in obstacle_markers:
        #    print(str(m.x) + ' ' + str(m.y))

        for obstacle in obstacle_markers:
            print('AMOUNT OF MARKERS PER OBSTACLE: ' + str(len(obstacle.markers)))
            # params that are the same for all parts of an obstacle:
            center_x = obstacle.x
            center_y = obstacle.y
            q_x = obstacle.markers[0].pose.orientation.x
            q_y = obstacle.markers[0].pose.orientation.y
            q_z = obstacle.markers[0].pose.orientation.z
            q_w = obstacle.markers[0].pose.orientation.w
            # Important: origin in rviz is bottom left, but on an image is always (0,0) => (x,y) -> (x,row_big-1-y)
            center_x_px = int((center_x - origin_x) / resolution) # x coordinate of the center of the whole obstacle
            center_y_px = int((center_y - origin_y) / resolution)
            x_px_rel_max = 0
            y_px_rel_max = 0
            for marker in obstacle.markers:
                # handle differently based on the type:
                if marker.type == 7: # 'sphere list'
                    counter_spheres += 1
                    # there is only one element in the points[] list
                    scale_x = marker.scale.x # relevant only for the sphere, for the polygon and the line it should be always 1.0; should be the same as scale.y
                    radius = scale_x/2 # Important: the circle is in a square scale_x * scale_y => radius = scale_x/2
                    radius_px = int(radius / resolution)
                    if radius_px > x_px_rel_max:
                        x_px_rel_max = y_px_rel_max = radius_px
                    center_x_px_part_obstacle = int(((center_x + marker.points[0].x) - origin_x) / resolution) # x coordinate of the center of the current circle part of an obstacle
                    center_y_px_part_obstacle = int(((center_y + marker.points[0].y) - origin_y) / resolution)
                    # the color of the obstacles could be taken from marker.color
                    cv2.circle(used_map_image,(center_x_px_part_obstacle,row_big-1-center_y_px_part_obstacle), radius_px, (0,0,255), 1) # a circle with red contours
                    #cv2.circle(used_map_image,(center_x_px,row_big-1-center_y_px), radius_px, (0,0,255), -1) # a red filled circle
                    #cv2.circle(used_map_image,(center_x_px,row_big-1-center_y_px), radius_px, (100,100,100), -1) # a grey filled circle
                elif marker.type == 11: # 'triangle list' = polygon
                    counter_polygons += 1
                    corners = marker.points
                    corners_array = []
                    for corner in corners:
                        x_temp = int(corner.x / resolution) + center_x_px # the points are relative to the center, so the origin should not be subtracted from them!
                        y_temp = int(corner.y / resolution) + center_y_px
                        corners_array.append([x_temp,row_big-1-y_temp])
                        if int(corner.x / resolution) > x_px_rel_max:
                            x_px_rel_max = int(corner.x / resolution)
                        if int(corner.y / resolution) > y_px_rel_max:
                            y_px_rel_max = int(corner.y / resolution)
                    pts = np.array([corners_array], np.int32)
                    pts = pts.reshape((-1,1,2))
                    cv2.polylines(used_map_image,[pts],True,(0,255,255)) # a polygon with yellow contours
                    #cv2.fillPoly(used_map_image, np.array([corners_array], np.int32), (0,255,255)) # a yellow filled polygon
                    #cv2.fillPoly(used_map_image, np.array([corners_array], np.int32), (100,100,100)) # a grey filled polygon
                elif marker.type == 5: # 'line list' =? the four room walls
                    counter_lines += 1
                    # for the line take only the first and last element of the array (TODO)
                    first_elem_x = marker.points[0].x
                    first_elem_y = marker.points[0].y
                    last_elem_X = marker.points[len(marker.points)-1].x
                    last_elem_y = marker.points[len(marker.points)-1].y
                    first_elem_x_px = int(first_elem_x / resolution) + center_x_px
                    first_elem_y_px = int(first_elem_y / resolution) + center_y_px
                    last_elem_x_px = int(last_elem_X / resolution) + center_x_px
                    last_elem_y_px = int(last_elem_y / resolution) + center_y_px
                    cv2.line(used_map_image,(first_elem_x_px,row_big-1-first_elem_y_px),(last_elem_x_px,row_big-1-last_elem_y_px),(255,0,0),10) # a blue line with thickness of 10 px
                else:
                    counter_others += 1
            
            # transformation from geometry_msgs/Pose2d to geometry_msgs/Pose (in 3d)
            # (from quaternions to Euler angles, where only yaw (a z-axis rotation) is interessting)
            siny_cosp = 2 * (q_w * q_z + q_x * q_y)
            cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            yaw_grad = math.degrees(yaw) # the same as the given 'theta' in the pose2d form in pedsim_test.py
            print('SPIN [deg]: ' + str(yaw_grad))

            # Important: consider the orientation -> rotation around the center of the obstacle (not a part of the obstacle) -> for example rotation around the center of the chair, so that the legs are in the right position!
            # rotate the whole obstacle with all of its parts at once:
            M = cv2.getRotationMatrix2D((center_x_px,row_big-1-center_y_px),yaw_grad,1) # create the transformation matrix (center, angle, scale); important: (0,0) top left corner
            dst = cv2.warpAffine(used_map_image,M,(col_big,row_big))
            # cut the rotated obstacle from the img (for now a rectangle around the obstacle) and upload it in the same position on top of the obstacle_map to update it with another obstacle
            x_distance = x_px_rel_max
            y_distance = y_px_rel_max
            # make it a rectangle, to be sure that the whole obstacle even after its rotation is still fully displayed
            # TODO NEXT: problem: the rotated objects (as a big rectangle no matter of their form) overlap and some pixels are therefore messed up
            # -> start with rotating the biggest obstacles!? or do not cut a rectangle but exactly the form of the object!?
            # -> somehow 'add' the pixels, so that a colored pixel is always above a white one!?
            if x_distance > y_distance: distance = x_distance
            else: distance = y_distance
            x_start = int(center_x_px - distance)
            y_start = int(center_y_px - distance)
            x_end = x_start+2*distance
            y_end = y_start+2*distance
            figure_img = dst[row_big-1-y_end:row_big-1-y_start, x_start:x_end]
            used_map_image[row_big-1-y_end:row_big-1-y_start, x_start:x_end] = figure_img
            cv2.imwrite("map_obstacles_part.png", figure_img)
            cv2.imwrite("map_obstacles.png", used_map_image)
            
        print('amount of obstacles: ' + str(amount_obstacles)) # 9 -> 142
        print('amount of spheres: ' + str(counter_spheres)) # 1 -> 43
        print('amount of polygons: ' + str(counter_polygons)) # 4 -> 95
        print('amount of lines: ' + str(counter_lines)) # 4  -> 4
        print('amount of others: ' + str(counter_others)) # 0
        # TODO: at the end all figures should be filled with a grey color and from there should be created an array with the values 'free'/'occupied'/'unknown'
        # -> and later on semantics should be added (sth like 'table1')
    
def callback_local_costmap(map_data):
    local_costmap_resolution = map_data.info.resolution # 0.05 # width: 666 [px] * 0.05 (resolution) = 33.3 [m]
    local_costmap_width = map_data.info.width # 60
    local_costmap_height = map_data.info.height # 60
    robot_position_x = map_data.info.origin.position.x
    robot_position_y = map_data.info.origin.position.y
    robot_orientation_z = map_data.info.origin.orientation.z # the orientation of the robot was not used, always z = 0 rad
    map_data_array = map_data.data
    map_data_length = len(map_data_array) # 3600
    print('LOCAL COSTMAP: \nlength: ' + str(map_data_length))

    # save the map as a grey image (black = free; grey = occupied; unknown = white):
    # the local costmap consits of a 60m x 60m block with the robot positioned in the middle
    map_data_array2 = np.array(map_data_array)
    map_reshaped = map_data_array2.reshape(local_costmap_height,local_costmap_width)
    print(map_reshaped)
    row,col = map_reshaped.shape
    temp = np.zeros((row,col))
    # the occupancy grid is in row-major order, starting with (0,0); our (0,0) is in the left down corner; for an image it is the upper left corner => mirror the pixels regarding the x axis to be right
    for i in range(row):
        for j in range(col):
            if(map_reshaped[i,j]==-1):
                temp[row-1-i,j]=255 # unknown = white
            else:
                temp[row-1-i,j]=map_reshaped[i,j] # black = free; grey = occupied
    #cv2.imshow("map_local_costmap_part", temp)
    cv2.imwrite("map_local_costmap_part.png", temp) # will be saved in folder $HOME\.ros # updates every time the robot moves
    #cv2.waitKey(0)

    # collect all local costmap data while the robot is moving and update the map image:
    # 1) create a black image with the size of the map image (at the beginning everything is black = 0 = free)
    # 2) from the local costmap 60x60 blocks, calculate the absolute coordinates in the map image
    # 3) update with the robot's movement the black image every time with the local 60x60 part
    # 4) always have the newest image saved (after the robot has moved in the whole space, every pixel should be definted to occupied/free/unknown)

    # Important: for rviz origin is on the bottom left, for an image is always on the top left; bottom left corner is for this map (-6,-6)! => corect the robot's position so that it is always positive
    # TODO: get params like resolution, origin etc. directly from the map yaml file instead of hard-coding them
    robot_position_x += 6
    robot_position_y += 6
    block_abs_width_left_rviz = int((robot_position_x / local_costmap_resolution) - (local_costmap_width / 2))
    block_abs_width_right_rviz = int((robot_position_x / local_costmap_resolution) + (local_costmap_width / 2))
    block_abs_height_bottom_rviz = int((robot_position_y / local_costmap_resolution) - (local_costmap_width / 2))
    block_abs_height_top_rviz = int((robot_position_y / local_costmap_resolution) + (local_costmap_width / 2))

    rospack = rospkg.RosPack()
    relative_img_path = os.path.join(rospack.get_path("simulator_setup"), "maps", "map_empty", "map_small.png")
    used_map_image = cv2.imread(relative_img_path) # get the size of the used map image: width x height 666 x 521
    row_big,col_big,val = used_map_image.shape

    # TODO NEXT: check again if the corners of the small images are correctly displayed or not
    # preferably there should be already a black image with this name and with the right dimensions in the $HOME\.ros folder,
    # if not - an image will be generated, but this takes time and in between the robot moves => some local costmap data can be missed and not be visualized
    my_file = Path("map_local_costmap.png")
    if not(my_file.is_file()): # the file does not exist
        temp_img = np.zeros((row_big,col_big)) # size of the big map image
    else:
        temp_img = cv2.imread("map_local_costmap.png")
        for i in range(col_big):
            if i > block_abs_height_bottom_rviz and i <= block_abs_height_top_rviz:
                for j in range(row_big):
                    if j > block_abs_width_left_rviz and j <= block_abs_width_right_rviz:
                        if(map_reshaped[i-block_abs_height_bottom_rviz-1,j-block_abs_width_left_rviz-1]==-1):
                            temp_img[row_big-1-i,j] = 255 # unknown = white
                        else:
                            if temp_img[row_big-1-i,j].all() == 0: # overwrite it only if it was before black (init status 'free'); if it was grey, leave it grey and do not overwrite it with black
                                if map_reshaped[i-block_abs_height_bottom_rviz-1,j-block_abs_width_left_rviz-1] == 100: # == 100 or > 90 # filter out the dark grey color -> leave the source (grey=~100) preferably without distortion
                                    temp_img[row_big-1-i,j] = map_reshaped[i-block_abs_height_bottom_rviz-1,j-block_abs_width_left_rviz-1] # black = free; grey = occupied
    #cv2.imshow("map_local_costmap", temp_img)
    cv2.imwrite("map_local_costmap.png", temp_img) # will be saved in folder $HOME\.ros
    #cv2.waitKey(0)
    # TODO: test it also when layer2 is visible for the laser scan
    # TODO: 'add' the image to the one from the /map topic
    # TODO NEXT: be sure to have both an image for visualization (black/white/gray) and an array (0,100,-1) for free/occupied/unknown!
    # -> it is maybe even faster, inside the iterations to save everything in an array and at the end to display the image

def callback_global_costmap(map_data):
    print('GLOBAL COSTMAP: ' + str(len(map_data.data))) # 346986 (the same length and info as the one from /map)

def callback_odom(odom_data):
    #print('(pos_x, pos_y, angle_z) = (' + str(odom_data.pose.pose.position.x) + ', ' + str(odom_data.pose.pose.position.y) + ', ' + str(odom_data.pose.pose.orientation.z) + ')')
    global robot_pos_x
    global robot_pos_y
    global robot_angle_z
    robot_pos_x = odom_data.pose.pose.position.x
    robot_pos_y = odom_data.pose.pose.position.y
    robot_angle_z = odom_data.pose.pose.orientation.z

def laser_scan_data_listener():
    rospy.init_node('scan_values')
    #time.sleep(2) # wait for the obstacles to be spawned
    # read the laser scan data and save also the absolute and relative position of the robot the whole time,
    # to be able to match it with the laser scan data; save also the info from the map to know where the obstacles are
    rospy.Subscriber("/scan", LaserScan, callback) # queue_size=1
    rospy.Subscriber('/map', OccupancyGrid, callback_map) # /move_base/global_costmap/costmap similar to /map
    #rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, callback_local_costmap)
    rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, callback_global_costmap)
    rospy.Subscriber('/odom', Odometry, callback_odom) # /odom returns position and velocity of the robot
    rospy.Subscriber('/flatland_markers', MarkerArray, callback_flatland_markers)
    rospy.spin()
    # TODO: use Daniel's GUI, because later we will need 1000 scenarios

if __name__ == '__main__':
    laser_scan_data_listener()
