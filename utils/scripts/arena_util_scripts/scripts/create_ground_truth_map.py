#!/usr/bin/env python
import rospy
import os
import math
import time
import rospkg
import cv2
import numpy as np
from numpy import asarray, savetxt
from collections import namedtuple
from visualization_msgs.msg import MarkerArray
from flatland_msgs.msg import DebugTopicList

amount_obstacles = 0
read_markers_info = 1
obstacle_names = [] # the whole name: 'model/table5_chair2_1'
RGB_color_ar = []
name_type_color_ar = [] # [NameTypeColorRef(name='table5_chair2_1', type='chair2', color=r: 0.3 g: 0.0 b: 0.1 a: 0.5), ...]
type_color_ar = [] # [TypeColorRef(type='chair1', color=r: 0.3 g: 0.0 b: 0.1 a: 0.5), ...]
c = 0

def callback_flatland_markers(markers_data): # get the ground truth map and array
    #print('MARKERS DATA: ' + str(markers_data))
    rospack = rospkg.RosPack()
    img_path = os.path.join(rospack.get_path("simulator_setup"), "maps", "map_empty", "map_small.png")
    used_map_image = cv2.imread(img_path)
    row_big,col_big,val = used_map_image.shape
    global read_markers_info, amount_obstacles, c
    amount_obstacles = len(markers_data.markers)
    c += 1
    print('TEST: ' + str(amount_obstacles) + ' ' + str(c))
    # TODO NEXT: it gets time for all obstacles (obstacle parts) to load, different for the different scenarios
    # -> time.sleep(10) is not helping
    # -> call this callback a lot (amount of calling = c) and only after that start with the calculations
    if c == 100 and read_markers_info == 1: # amount_obstacles == 142 # c == 100
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
                if not(int(center_x) >= -2 and int(center_x) <= 2 and int(center_y) >= -2 and int(center_y) <= 2): # TODO: try to get the robot out (it should be at (0,0) but maybe it moved) -> get its position from /flatland_server/debug/model/myrobot !?
                    if not(center_x == -6 and center_y == -6): # for the type 'line' with center (-6,-6)=origin !? (TODO)
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

            # transformation from geometry_msgs/Pose2d to geometry_msgs/Pose (in 3d)
            # (from quaternions to Euler angles, where only yaw (a z-axis rotation) is interessting)
            siny_cosp = 2 * (q_w * q_z + q_x * q_y)
            cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            yaw_grad = math.degrees(yaw) # the same as the given 'theta' in the pose2d form in pedsim_test.py
            print('SPIN [deg]: ' + str(yaw_grad)) # the angle with which the obstacle should be rotated

            # test image per obstacle to avoid overlapping afer obstacle rotation; here the obstacles will be rotated before being placed on the final image 'used_map_image' that should have all obstacles at the end
            # necessary to avoid the problems: while cutting an obstacle to take parts of other obstacles and while putting the obstacle to overwrite with white other obstacles
            temp_image_for_obstacle_rotations = cv2.imread(img_path) # the map image without obstacles

            for marker in obstacle.markers:
                # color the obstacles in the ground truth map with the same color from rviz (from their definition in the yaml file)
                # the color from marker.color is scaled between 0 and 1, for opencv it should be between 0 and 255
                # TODO: the color is visualized differently (rviz vs. opencv)?
                r = int(marker.color.r*255)
                g = int(marker.color.g*255)
                b = int(marker.color.b*255)
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
                    #cv2.circle(temp_image_for_obstacle_rotations,(center_x_px_part_obstacle,row_big-1-center_y_px_part_obstacle), radius_px, (0,0,255), 1) # a circle with red contours
                    #cv2.circle(temp_image_for_obstacle_rotations,(center_x_px,row_big-1-center_y_px), radius_px, (0,0,255), -1) # a red filled circle
                    #cv2.circle(temp_image_for_obstacle_rotations,(center_x_px,row_big-1-center_y_px), radius_px, (100,100,100), -1) # a grey filled circle
                    cv2.circle(temp_image_for_obstacle_rotations,(center_x_px,row_big-1-center_y_px), radius_px, (r,g,b), -1) # a filled circle with the original color of the obstacle
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
                    #cv2.polylines(temp_image_for_obstacle_rotations,[pts],True,(0,255,255)) # a polygon with yellow contours
                    #cv2.fillPoly(temp_image_for_obstacle_rotations, np.array([corners_array], np.int32), (0,255,255)) # a yellow filled polygon
                    #cv2.fillPoly(temp_image_for_obstacle_rotations, np.array([corners_array], np.int32), (100,100,100)) # a grey filled polygon
                    cv2.fillPoly(temp_image_for_obstacle_rotations, np.array([corners_array], np.int32), (r,g,b)) # a filled polygon with the original color of the obstacle
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
                    #cv2.line(temp_image_for_obstacle_rotations,(first_elem_x_px,row_big-1-first_elem_y_px),(last_elem_x_px,row_big-1-last_elem_y_px),(255,0,0),10) # a blue line with thickness of 10 px
                    #cv2.line(temp_image_for_obstacle_rotations,(first_elem_x_px,row_big-1-first_elem_y_px),(last_elem_x_px,row_big-1-last_elem_y_px),(100,100,100),10) # a grey line with thickness of 10 px
                    cv2.line(temp_image_for_obstacle_rotations,(first_elem_x_px,row_big-1-first_elem_y_px),(last_elem_x_px,row_big-1-last_elem_y_px),(r,g,b),10) # a line with thickness of 10 px with the original color of the obstacle
                else:
                    counter_others += 1

            # Important: consider the orientation -> rotation around the center of the obstacle (not a part of the obstacle) -> for example rotation around the center of the chair, so that the legs are in the right position!
            # rotate the whole obstacle with all of its parts at once:
            M = cv2.getRotationMatrix2D((center_x_px,row_big-1-center_y_px),yaw_grad,1) # create the transformation matrix (center, angle, scale); important: (0,0) top left corner
            dst = cv2.warpAffine(temp_image_for_obstacle_rotations,M,(col_big,row_big))
            # cut a rectangle around the rotated obstacle from the img and upload it in the same position on top of the obstacle_map to update it with another obstacle
            x_distance = x_px_rel_max # before the rotation
            y_distance = y_px_rel_max
            # make it a rectangle, to be sure that the whole obstacle even after its rotation is still fully displayed
            if x_distance > y_distance: distance = x_distance
            else: distance = y_distance
            # Important: 'distance' is the max length of the rectangle around the not yet rotated obstacle => idea: take the diagonal (should work also if the obstacle was a polygon)
            # Idea: take as less as possible pixels but also at 100% enough so that the whole obstacle is there, at the end all white pixels will be filtert out
            distance_diagonal = int(math.sqrt(2*(math.pow(distance,2))))
            x_start = int(center_x_px - distance_diagonal)
            y_start = int(center_y_px - distance_diagonal)
            x_end = x_start+2*distance_diagonal
            y_end = y_start+2*distance_diagonal

            # write the colored and rotated obstacle to the image
            figure_img = dst[row_big-1-y_end:row_big-1-y_start, x_start:x_end] # just a step in between for debugging
            for r in range(row_big-1-y_end, row_big-1-y_start):
                for c in range(x_start, x_end):
                    RGB_color_dst_ar = [dst[r, c, 0], dst[r, c, 1], dst[r, c, 2]]
                    white_ar = [255,255,255]
                    if RGB_color_dst_ar != white_ar: # filter out the white pixels to get only the obstacle contours to prevent overlapping
                        used_map_image[r, c] = dst[r, c]
                    # Info: because of rotation some of the colors are not that perfect any more (for example red is not only (255,0,0) etc.) 
            cv2.imwrite("map_obstacles_part.png", figure_img)
            cv2.imwrite("map_obstacles.png", used_map_image)
            
        print('amount of obstacles: ' + str(amount_obstacles)) # 9 -> 142
        print('amount of spheres: ' + str(counter_spheres)) # 1 -> 43
        print('amount of polygons: ' + str(counter_polygons)) # 4 -> 95
        print('amount of lines: ' + str(counter_lines)) # 4  -> 4
        print('amount of others: ' + str(counter_others)) # 0

        # make sure that all obstacles are marked grey
        # make some color changes, so that 100 = occupied = grey, 0 = free = black, -1 = unknown = white, like in map_topic.png
        # => here in map_obstacles.png grey stays grey = 100 = occupied, black -> grey, white -> black, some other color -> white (or maybe directly grey?)
        ground_truth_map = cv2.imread("map_obstacles.png")
        white_ar = [255,255,255]
        grey_ar = [100,100,100]
        black_ar = [0,0,0]
        for i in range(ground_truth_map.shape[0]):
            for j in range(ground_truth_map.shape[1]):
                RGB_color_dst_ar = [ground_truth_map[i, j, 0], ground_truth_map[i, j, 1], ground_truth_map[i, j, 2]]
                if RGB_color_dst_ar == grey_ar:
                    pass
                elif RGB_color_dst_ar == black_ar:
                    ground_truth_map[i, j] = (100,100,100)
                elif RGB_color_dst_ar == white_ar:
                    ground_truth_map[i, j] = (0,0,0)
                else:
                    #ground_truth_map[i, j] = (255,255,255) # they are some 'unknown' points because of the rotation and therefore not that super clear visualization of some obstacles
                    ground_truth_map[i, j] = (100,100,100) # we know that they are part of the obstacle, so it is maybe better that the groundtruth map/array do not have 'unknown' points
        cv2.imwrite("map_ground_truth.png", ground_truth_map)
        print('GROUND TRUTH MAP DONE!')

        # create a ground truth array (in row-major order) with the values 100/0/-1 from the ground truth map with values black/grey/white
        # Important: for an image the start (0,0) is in the upper left corner (see 'ground_truth_array_image_order'), for the simulation (occupancy grid for example) is start (0,0) in the down left corner -> both versions are just mirrored to one another regarding the x axis
        ground_truth_array_image_order = []
        ground_truth_array_sim_order = []
        for i in range(ground_truth_map.shape[0]):
            for j in range(ground_truth_map.shape[1]):
                RGB_color_dst_ar = [ground_truth_map[i, j, 0], ground_truth_map[i, j, 1], ground_truth_map[i, j, 2]]
                if(RGB_color_dst_ar==white_ar):
                    ground_truth_array_image_order.append(-1) # 255 = white = unknown = -1
                elif(RGB_color_dst_ar==grey_ar):
                    ground_truth_array_image_order.append(100) # 100 = grey = occupied = 100
                else:
                    ground_truth_array_image_order.append(0) # 0 = black = free = 0
        i = ground_truth_map.shape[0] - 1
        while i >=0:
            for j in range(ground_truth_map.shape[1]):
                RGB_color_dst_ar = [ground_truth_map[i, j, 0], ground_truth_map[i, j, 1], ground_truth_map[i, j, 2]]
                if(RGB_color_dst_ar==white_ar):
                    ground_truth_array_sim_order.append(-1) # 255 = white = unknown = -1
                elif(RGB_color_dst_ar==grey_ar):
                    ground_truth_array_sim_order.append(100) # 100 = grey = occupied = 100
                else:
                    ground_truth_array_sim_order.append(0) # 0 = black = free = 0
            i -= 1
        print('GROUND TRUTH ARRAY DONE!') # both arrays with length of 346986
        # save the arrays to files
        ground_truth_ar_image_order = asarray([ground_truth_array_image_order])
        savetxt('ground_truth_img_order.csv', ground_truth_ar_image_order, delimiter=',')
        ground_truth_ar_sim_order = asarray([ground_truth_array_sim_order])
        savetxt('ground_truth_sim_order.csv', ground_truth_ar_sim_order, delimiter=',')

        # TODO NEXT: add semantics ('table1'/..) to the ground truth map and array -> different color per obstacle for the map and different type for the array
        # -> from the obstacle color and the corresponding color-type table get the obstacle type and save it in the ground truth array -> array with two entries per pixel: occupied or not and obstacle type/color
        # -> maybe even write the type in rviz like a label on top of the obstacles

def callback_flatland_server(flatland_server_data):
    global obstacle_names
    for topic in flatland_server_data.topics:
        if "model/" in topic and "model/myrobot" not in topic:
            #print(topic)
            name_unique = 1
            for name in obstacle_names:
                if (name == topic):
                    name_unique = 0
            if name_unique == 1:
                obstacle_names.append(topic)
    #print(obstacle_names) # ['model/table1', ...]
    #print(len(obstacle_names)) # 26 # they are loading one by one! so wait until the last one is loaded
    
    global name_type_color_ar, type_color_ar
    NameTypeColorRef = namedtuple("NameTypeColorRef", "name type color")
    TypeColorRef = namedtuple("TypeColorRef", "type color")
    count = 0
    for topic_sub_name in obstacle_names:
        #rospy.Subscriber('/flatland_server/debug/' + str(topic_sub_name), MarkerArray, callback)
        msg = rospy.wait_for_message('/flatland_server/debug/' + str(topic_sub_name), MarkerArray)
        RGB_color = msg.markers[0].color
        # obstacle name's: table types = 'model/table[nummer]', chair types = 'model/table[nummer]_chair[nummer]_counter'
        # => split the name: without 'model/' and for the chairs leave only the middle part 'chair_nummer'
        # => example: topic_sub_name = 'model/table5_chair2_1', name = 'table5_chair2_1' and type = 'chair2'
        name = topic_sub_name.split("model/")[1]
        type = name
        if "chair" in type:
            type = type.split("_")[1]
        name_unique = type_unique = 1
        for name_color_ref in name_type_color_ar:
            if name_color_ref.name == name: name_unique = 0
        if name_unique == 1:
            name_type_color_ar.append(NameTypeColorRef(name, type, RGB_color)) # add only once per obstacle
        for type_color_ref in type_color_ar:
            if type_color_ref.type == type: type_unique = 0
        if type_unique == 1:
            type_color_ar.append(TypeColorRef(type, RGB_color)) # add only once per obstacle type
        count += 1
    #print(name_type_color_ar) # [NameTypeColorRef(name='table1_chair1_1', type='chair1', color=r: 0.3 g: 0.0 b: 0.1 a: 0.5), ...]
    #print(len(name_type_color_ar)) # = len(obstacle_names) # 26 # they are loading one by one! so wait until the last one is loaded
    #print(type_color_ar) # [TypeColorRef(type='chair1', color=r: 0.3 g: 0.0 b: 0.1 a: 0.5), ...]
    #print(len(type_color_ar)) # != len(obstacle_names) # 7

    # print and save to a .txt file the obstacle's type-color correspondance table
    table = '\tobstacle_type\tRGB_color'
    for elem in type_color_ar:
        table += '\n\t' + str(elem.type) + '\t\t(' + str(elem.color.r) + ',' + str(elem.color.g) + ',' + str(elem.color.b) + ')'
    separator = '/**************************************************************/'
    print(separator + '\n' + table + '\n' + separator)
    with open('table.txt', 'w') as f: # overwrite the content
        f.write(table)

def create_ground_truth_data():
    rospy.init_node('ground_truth_data')
    #time.sleep(2) # wait for the obstacles to be spawned
    #rospy.Subscriber('/flatland_server/debug/model/table1', MarkerArray, callback)
    rospy.Subscriber('/flatland_server/debug/topics', DebugTopicList, callback_flatland_server)
    rospy.Subscriber('/flatland_markers', MarkerArray, callback_flatland_markers)
    rospy.spin()

if __name__ == '__main__':
    # the calculations and iterations here are moved to a separate node, since they are slowing the data collection (laser scan etc.)
    create_ground_truth_data()
