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
read_flatland_info = 1
obstacle_names = [] # the whole name: 'model/table5_chair2_1'
RGB_color_ar = []
name_id_type_color_ar = [] # [NameIDTypeColorRef(name='7_table5_chair2_1', id=7, type='chair2', color=r: 0.3 g: 0.0 b: 0.1 a: 0.5), ...]
id_type_color_ar = [] # [IDTypeColorRef(id=7, type='chair1', color=r: 0.3 g: 0.0 b: 0.1 a: 0.5), ...]
c = 0
robot_cur_pos_x = robot_cur_pos_y = robot_cur_pos_z = 0
robot_color_r = robot_color_g = robot_color_b = 0

def callback_flatland_markers(markers_data): # get the ground truth map and array
    #print('MARKERS DATA: ' + str(markers_data))
    rospack = rospkg.RosPack()
    img_path = os.path.join(rospack.get_path("simulator_setup"), "maps", "map_empty", "map_small.png")
    used_map_image = cv2.imread(img_path)
    row_big,col_big,val = used_map_image.shape
    global read_markers_info, amount_obstacles, c
    amount_obstacles = len(markers_data.markers)
    c += 1
    counter_spheres = 0
    counter_polygons = 0
    counter_others = 0
    counter_lines = 0
    origin_x = -6
    origin_y = -6
    resolution = 0.05
    obstacle_markers = []
    ObstacleMarker = namedtuple("ObstacleMarker", "x y markers") # a struct per obstacle (center_x, center_y, markers_array)

    # Important: if this FOR loop is before the IF statement checking the amount of obstacles, then '~obstacles_amount' should be = the amount of obstacles (26 for scenario 1), but if it is inside of the IF statement, then '~obstacles_amount' should be = all parts of all obstacles (133 for scenario 1 + 9 for the walls and the robot, so al together 142)
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
            # Important: do not include the robot in the ground truth map
            # -> at the beginning the robot should be at (0,0) [idea1], but maybe it moved before the data was colected (get its position from /flatland_server/debug/model/myrobot) [idea2]
            # -> or filter out its green rgb color [idea3] (no obstacle is then allowed to have this color!)
            # -> to be 100% sure, just run this node (for ground truth data colection) when the robot is not moving [idea4]
            #tolerance = 2 # in px
            #if not(int(center_x) >= -tolerance and int(center_x) <= tolerance and int(center_y) >= -tolerance and int(center_y) <= tolerance): # idea1
            #if not(int(center_x) >= robot_cur_pos_x-tolerance and int(center_x) <= robot_cur_pos_x+tolerance and int(center_y) >= robot_cur_pos_y-tolerance and int(center_y) <= robot_cur_pos_y+tolerance): # idea2
            if not(marker.color.r == robot_color_r and marker.color.g == robot_color_g and marker.color.b == robot_color_b): # idea3
                if not(center_x == origin_x and center_y == origin_y): # filter out also the type 'line' with center=origin (the walls)
                    part_markers = [marker]
                    obstacle_markers.append(ObstacleMarker(center_x, center_y, part_markers)) # ({'x': center_x, 'y': center_y, 'markers': part_markers})
    print('AMOUNT OF OBSTACLES: ' + str(len(obstacle_markers))) # 26 for scenario 1

    #print('Loaded obstacles for the current callback id: ' + str(amount_obstacles) + ' ' + str(c))
    # Important: it gets time for all obstacles (obstacle parts) to load, different for the different scenarios
    # -> Idea1: call this callback a lot (amount of calling = c) and only after that start with the calculations: easier way, since the obstacle parts do not have to be calculated, but slower and there is no guarantee that it is enough for other bigger scenarios
    # -> Idea2: set the parameter 'obstacles_amount' to the amount of obstacle parts for the current scenario by launching: all parts should be calculated, but once this is done, it will work with every scenario, also faster
    node_obstacles_amount = rospy.get_param('~obstacles_amount') # roslaunch arena_bringup pedsim_test.launch obstacles_amount:=142
    #if c == 100 and read_markers_info == 1: # Idea1
    #if amount_obstacles == node_obstacles_amount and read_markers_info == 1: # Idea2
    if len(obstacle_markers) == node_obstacles_amount and read_markers_info == 1 and len(obstacle_names) == node_obstacles_amount: # Idea2.2 consider the len(obstacle_markers) instead of amount_obstacles (also make sure that the correspondance color-type table is already ready, since this could also take a while)
        print('AMOUNT OF OBSTACLES: ' + str(len(obstacle_markers))) # 26 for scenario 1
        read_markers_info = 0
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
                r = int(marker.color.r*255)
                g = int(marker.color.g*255)
                b = int(marker.color.b*255)
                # parameter 'a' is also available, but for the ground truth map not significant
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
                    cv2.circle(temp_image_for_obstacle_rotations,(center_x_px,row_big-1-center_y_px), radius_px, (b,g,r), -1) # a filled circle with the original color of the obstacle
                    #cv2.circle(temp_image_for_obstacle_rotations,(center_x_px,row_big-1-center_y_px), radius_px, (b,g,r), 2) # a circle with the original color of the obstacle
                    # Important: the color in opencv is in order BGR!
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
                    cv2.fillPoly(temp_image_for_obstacle_rotations, np.array([corners_array], np.int32), (b,g,r)) # a filled polygon with the original color of the obstacle
                    #cv2.polylines(temp_image_for_obstacle_rotations, [pts], True, (b,g,r)) # a polygon with the original color of the obstacle
                elif marker.type == 5: # 'line list' = the room walls?; 4 lines; TODO: filtered out anyway, but maybe needed, so that the walls get also an assigned obstacle color/id!?
                    counter_lines += 1
                    # TODO: since an example consists of like ~25 points, it may also include edges, so better take all points in a loop then just the forst and last one
                    # for the line take for now only the first and last element of the array
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
                    cv2.line(temp_image_for_obstacle_rotations,(first_elem_x_px,row_big-1-first_elem_y_px),(last_elem_x_px,row_big-1-last_elem_y_px),(b,g,r),10) # a line with thickness of 10 px with the original color of the obstacle
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
            tolerance = 5 # [px] Important: it helps for example for the rotated square table! even a small tolerance (like 1 or 2) is enough but make it slightly bigger just to be sure it works correctly, but also not too much (so that it does not go over the borders after the rotation!)
            x_start = int(center_x_px - distance_diagonal) - tolerance
            y_start = int(center_y_px - distance_diagonal) - tolerance
            x_end = x_start+2*distance_diagonal + 2*tolerance
            y_end = y_start+2*distance_diagonal + 2*tolerance

            # write the colored and rotated obstacle to the image (better take only the part with the image on it as well as a small tolerance around then take the whole image without the borders, it's much more faster and still works every time; also taking the whole image could be wrong since the image has been rotated and some black parts are now on it now)
            figure_img = dst[row_big-1-y_end:row_big-1-y_start, x_start:x_end] # just a step in between for debugging
            for r in range(row_big-1-y_end, row_big-1-y_start):
                for c in range(x_start, x_end):
                    RGB_color_dst_ar = [dst[r, c, 2], dst[r, c, 1], dst[r, c, 0]]
                    white_ar = [255,255,255]
                    if RGB_color_dst_ar != white_ar: # filter out the white pixels to get only the obstacle contours to prevent overlapping
                        used_map_image[r, c] = dst[r, c]
                    # Info: because of rotation some of the colors are not that perfect any more (for example red is not only (255,0,0) etc.) 
            cv2.imwrite("map_obstacles_part_debug.png", figure_img)
            #cv2.imwrite("map_obstacles_part_debug_" + str(obstacle.x) + "_" + str(obstacle.y) + ".png", figure_img) # good for DEBUGGING
            cv2.imwrite("map_obstacles.png", used_map_image)
            
        # for debugging:
        print('amount of obstacles: ' + str(amount_obstacles)) # 9 -> 142
        print('amount of spheres: ' + str(counter_spheres)) # 1 -> 43
        print('amount of polygons: ' + str(counter_polygons)) # 4 -> 95
        print('amount of lines: ' + str(counter_lines)) # 4  -> 4
        print('amount of others: ' + str(counter_others)) # 0

        # from the map with colorful obstacles 'used_map_image' (white: free; color: occupied, obstacles; black: borders, walls)
        # create a ground truth map with the characteristics of an occupancy grid (0 = free = black, 100 = occupied = grey), like in map_topic.png
        # and a ground truth semantic map (0 = free = black, int>0 (semantics) = occupied = RGBcolor (still could be grey, but also another color))
        # => grey stays grey = 100 = occupied, black -> grey, white -> black, some other color -> grey or preserve the color
        ground_truth_map = cv2.imread("map_obstacles.png")
        ground_truth_semantic_map = cv2.imread("map_obstacles.png")
        white_ar = [255,255,255] # in RGB and in BGR the same
        grey_ar = [100,100,100] # in RGB and in BGR the same
        black_ar = [0,0,0] # in RGB and in BGR the same

        # at the same time create also a ground truth array/s (in row-major order) from the ground truth map/s
        # Important: for an image the start (0,0) is in the upper left corner (see 'ground_truth_array_image_order'), for the simulation (occupancy grid for example) is start (0,0) in the down left corner -> both versions are just mirrored to one another regarding the x axis
        # 4 different ground truth arrays:
        ground_truth_array_image_order = [] # (0,0) is upper left corner; black = 0 = free, grey = 100 = occupied; no white = -1 = unknown, since the ground truth should not have unknown parts
        ground_truth_array_sim_order = [] # (0,0) is down left corner; black = 0 = free, grey = 100 = occupied; no white = -1 = unknown, since the ground truth should not have unknown parts
        ground_truth_array_semantic_image_order = [] # (0,0) is upper left corner; black = 0 = free; color = >0 = occupied (100 are the walls); no white = -1 = unknown, since the ground truth should not have unknown parts
        ground_truth_array_semantic_sim_order = [] # (0,0) is down left corner; lack = 0 = free; color = >0 = occupied (100 are the walls); no white = -1 = unknown, since the ground truth should not have unknown parts
        
        for i in range(ground_truth_map.shape[0]):
            for j in range(ground_truth_map.shape[1]):
                # Important: the color in opencv is in order BGR, so convert it to get RGB
                RGB_color_grey = [ground_truth_map[i, j, 2], ground_truth_map[i, j, 1], ground_truth_map[i, j, 0]]
                RGB_color_colorful = [ground_truth_semantic_map[i, j, 2], ground_truth_semantic_map[i, j, 1], ground_truth_semantic_map[i, j, 0]]
                if RGB_color_grey == grey_ar:
                    ground_truth_array_image_order.append(100) # 100 = grey = occupied = 100
                    ground_truth_array_semantic_image_order.append(100) # 100 = grey = occupied = 100
                elif RGB_color_grey == black_ar:
                    ground_truth_map[i, j] = (100,100,100) # grey
                    ground_truth_semantic_map[i, j] = (100,100,100) # grey
                    ground_truth_array_image_order.append(100) # 100 = grey = occupied = 100
                    ground_truth_array_semantic_image_order.append(100) # 100 = grey = occupied = 100
                elif RGB_color_grey == white_ar:
                    ground_truth_map[i, j] = (0,0,0) # black
                    ground_truth_semantic_map[i, j] = (0,0,0) # black
                    ground_truth_array_image_order.append(0) # 0 = black = free = 0
                    ground_truth_array_semantic_image_order.append(0) # 0 = black = free = 0
                else: # other color (obstacle or its unclear border parts because of rotation)
                    # -> do not mark the border black since info will be lost
                    # -> do not choose white since everything in the ground truth should be known
                    # -> grey borders of the colored obstacles might be a problem for recognising the type of the obstacles
                    # -> finding the closest color in the table to the one form the borders, so to have a color tolerance, also do not works, since the color is way too different and it could lead to coloring wrongly
                    # => so color the borders with the same color as the obstacle itself: to do this first color the borders white and later in another loop color the white px with their colorful neighbours px color
                    color_temp_grey = 0 # black per default
                    color_temp_r_colorful = color_temp_g_colorful = color_temp_b_colorful = 255 # white per default to know if an obstacle was perfectly colored or not, when everything is perfect, there should be no white pixels
                    id_temp = -1 # white = unknown per default
                    # for the semantics, having the obstacle color the type/id of the obstacle should be taken form the id-type-color-table:
                    for elem in id_type_color_ar:
                        if int(elem.color.r*255) == RGB_color_colorful[0] and int(elem.color.g*255) == RGB_color_colorful[1] and int(elem.color.b*255) == RGB_color_colorful[2]:
                            color_temp_r_colorful = RGB_color_colorful[0]
                            color_temp_g_colorful = RGB_color_colorful[1]
                            color_temp_b_colorful = RGB_color_colorful[2]
                            color_temp_grey = 100
                            id_temp = elem.id
                            break
                    ground_truth_semantic_map[i, j] = (color_temp_b_colorful, color_temp_g_colorful, color_temp_r_colorful) # black or color # again coloring with opencv needs the BGR form!
                    ground_truth_array_semantic_image_order.append(id_temp) # index > 0 or -1
                    ground_truth_map[i, j] = (color_temp_grey,color_temp_grey,color_temp_grey) # black or grey
                    ground_truth_array_image_order.append(color_temp_grey) # black or grey

        dist = 3 # px # 1 px works for almost every case, but to be sure make it 2-3px, also not bigger since overlapping from other near obstacles may occur
        for i in range(ground_truth_semantic_map.shape[0]):
            for j in range(ground_truth_semantic_map.shape[1]):
                BGR_color = [ground_truth_semantic_map[i, j, 0], ground_truth_semantic_map[i, j, 1], ground_truth_semantic_map[i, j, 2]]
                if BGR_color == white_ar: # if it is white colored, then it belongs to a border => update the color
                    # Idea1: find the color of the obstacle the border belongs to -> take the color from the pixel next to it (above/left/right/bottom), if it is not the same color or black
                    # Idea2 (TODO): take the info from position/orientation/size etc. instead !?
                    BGR_color_left_neighbour = [ground_truth_semantic_map[i-dist, j, 0], ground_truth_semantic_map[i-dist, j, 1], ground_truth_semantic_map[i-dist, j, 2]]
                    BGR_color_right_neighbour = [ground_truth_semantic_map[i+dist, j, 0], ground_truth_semantic_map[i+dist, j, 1], ground_truth_semantic_map[i+dist, j, 2]]
                    BGR_color_bottom_neighbour = [ground_truth_semantic_map[i, j-dist, 0], ground_truth_semantic_map[i, j-dist, 1], ground_truth_semantic_map[i, j-dist, 2]]
                    BGR_color_upper_neighbour = [ground_truth_semantic_map[i, j+dist, 0], ground_truth_semantic_map[i, j+dist, 1], ground_truth_semantic_map[i, j+dist, 2]]
                    if BGR_color_left_neighbour!=white_ar and BGR_color_left_neighbour!=black_ar:
                        ground_truth_semantic_map[i, j] = (BGR_color_left_neighbour[0], BGR_color_left_neighbour[1], BGR_color_left_neighbour[2])
                    if BGR_color_right_neighbour!=white_ar and BGR_color_right_neighbour!=black_ar:
                        ground_truth_semantic_map[i, j] = (BGR_color_right_neighbour[0], BGR_color_right_neighbour[1], BGR_color_right_neighbour[2])
                    if BGR_color_bottom_neighbour!=white_ar and BGR_color_bottom_neighbour!=black_ar:
                        ground_truth_semantic_map[i, j] = (BGR_color_bottom_neighbour[0], BGR_color_bottom_neighbour[1], BGR_color_bottom_neighbour[2])
                    if BGR_color_upper_neighbour!=white_ar and BGR_color_upper_neighbour!=black_ar:
                        ground_truth_semantic_map[i, j] = (BGR_color_upper_neighbour[0], BGR_color_upper_neighbour[1], BGR_color_upper_neighbour[2])
                    #print(ground_truth_semantic_map[i, j]) # for debugging: [100 100 100], [178   0 127], [127  25   0] etc.
                    # also update the id:
                    for elem in id_type_color_ar:
                        if int(elem.color.r*255) == ground_truth_semantic_map[i, j, 0] and int(elem.color.g*255) == ground_truth_semantic_map[i, j, 1] and int(elem.color.b*255) == ground_truth_semantic_map[i, j, 2]:
                            id_temp = elem.id
                            break
                    #ground_truth_array_semantic_image_order[i,j] = id_temp # make only one dimension from i and j
                    ground_truth_array_semantic_image_order[(i-1)*ground_truth_semantic_map.shape[1]+j] = id_temp # index > 0

        cv2.imwrite("map_ground_truth.png", ground_truth_map)
        cv2.imwrite("map_ground_truth_semantic.png", ground_truth_semantic_map)
        print('GROUND TRUTH MAP AND ARRAY IMG ORDER DONE!')

        # TODO: create a white-black map like "map_small.png", where the black areas are obstacles (needed to be uploaded in the GUI for creating scenarios)

        #ground_truth_map here is already only in black and grey, so for the color take the info from ground_truth_semantic_map
        i = ground_truth_map.shape[0] - 1
        while i >=0:
            for j in range(ground_truth_map.shape[1]):
                BGR_color_grey = [ground_truth_map[i, j, 0], ground_truth_map[i, j, 1], ground_truth_map[i, j, 2]]
                BGR_color_colorful = [ground_truth_semantic_map[i, j, 0], ground_truth_semantic_map[i, j, 1], ground_truth_semantic_map[i, j, 2]]
                if(BGR_color_grey==black_ar): # black
                    ground_truth_array_sim_order.append(0) # 0 = black = free
                    ground_truth_array_semantic_sim_order.append(0) # 0 = black = free
                else: # grey
                    color_temp_grey = 0 # black per default
                    color_temp_colorful = 0 # black per default
                    for elem in id_type_color_ar:
                        if int(elem.color.r*255) == BGR_color_colorful[2] and int(elem.color.g*255) == BGR_color_colorful[1] and int(elem.color.b*255) == BGR_color_colorful[0]:
                            color_temp_colorful = elem.id
                            color_temp_grey = 100
                            break
                    ground_truth_array_sim_order.append(color_temp_grey)
                    ground_truth_array_semantic_sim_order.append(color_temp_colorful)
            i -= 1
        print('GROUND TRUTH ARRAY SIM ORDER DONE!')
        # TODO: vizualise the arrays just for debugging to see if everything is correct
        # TODO idea: it is maybe even faster, inside the iterations to save everything in an array and at the end to display the image

        # save the arrays to files (all with length of 346986)
        savetxt('ground_truth_img_order.csv', asarray([ground_truth_array_image_order]), delimiter=',')
        savetxt('ground_truth_sim_order.csv', asarray([ground_truth_array_sim_order]), delimiter=',')
        savetxt('ground_truth_semantic_img_order.csv', asarray([ground_truth_array_semantic_image_order]), delimiter=',')
        savetxt('ground_truth_semantic_sim_order.csv', asarray([ground_truth_array_semantic_sim_order]), delimiter=',')

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
    #print(obstacle_names) # ['model/1_table1', ...]
    #print(len(obstacle_names)) # 26 # they are loading one by one! so wait until the last one is loaded

    global name_id_type_color_ar, id_type_color_ar
    NameIDTypeColorRef = namedtuple("NameIDTypeColorRef", "name id type color")
    IDTypeColorRef = namedtuple("IDTypeColorRef", "id type color")
    count = 0
    for topic_sub_name in obstacle_names:
        #rospy.Subscriber('/flatland_server/debug/' + str(topic_sub_name), MarkerArray, callback)
        msg = rospy.wait_for_message('/flatland_server/debug/' + str(topic_sub_name), MarkerArray)
        RGB_color = msg.markers[0].color
        # obstacle name's: table types = 'model/[global_nummer]_table[nummer]_counter', chair types = 'model/[global_nummer]_table[nummer]_chair[nummer]_counter'
        # => split the name: take only the [global_nummer] for the id; without 'model/[global_nummer]_' and '_counter' for the tables and for the chairs leave only the middle part 'chair[nummer]'
        # => example: topic_sub_name = 'model/5_table5_chair2_1', name = 'table5_chair2_1', type = 'chair2' and id = 5
        name = topic_sub_name.split("model/")[1]
        id = int(name.split("_")[0])
        if "chair" in name: type = name.split("_")[2]
        else: type = name.split("_")[1]
        name_unique = type_unique = 1
        for name_color_ref in name_id_type_color_ar:
            if name_color_ref.name == name: name_unique = 0
        if name_unique == 1:
            name_id_type_color_ar.append(NameIDTypeColorRef(name, id, type, RGB_color)) # add only once per obstacle
        for type_color_ref in id_type_color_ar:
            if type_color_ref.type == type: type_unique = 0
        if type_unique == 1:
            id_type_color_ar.append(IDTypeColorRef(id, type, RGB_color)) # add only once per obstacle type
        count += 1
    #print(name_id_type_color_ar) # [NameIDTypeColorRef(name='7_table1_chair1_1', id=7, type='chair1', color=r: 0.3 g: 0.0 b: 0.1 a: 0.5), ...]
    #print(len(name_id_type_color_ar)) # = len(obstacle_names) # 26 # they are loading one by one! so wait until the last one is loaded
    #print(id_type_color_ar) # [TypeColorRef(id=7, type='chair1', color=r: 0.3 g: 0.0 b: 0.1 a: 0.5), ...]
    #print(len(id_type_color_ar)) # != len(obstacle_names) # 7

    # print and save to a .txt file the obstacle's type-color correspondance table
    table = '\tid\tobstacle_type\tRGB_color'
    for elem in id_type_color_ar:
        table += '\n\t' + str(elem.id) + '\t' + str(elem.type) + '\t\t(' + str(elem.color.r) + ',' + str(elem.color.g) + ',' + str(elem.color.b) + ')'
    separator = '/**************************************************************/'
    print(separator + '\n' + table + '\n' + separator)
    with open('table.txt', 'w') as f: # overwrite the content
        f.write(table)

def callback_myrobot(data):
    global robot_cur_pos_x, robot_cur_pos_y, robot_cur_pos_z, robot_color_r, robot_color_g, robot_color_b
    robot_cur_pos_x = data.markers[0].pose.position.x
    robot_cur_pos_y = data.markers[0].pose.position.y
    robot_cur_pos_z = data.markers[0].pose.position.z
    robot_color_r = data.markers[0].color.r
    robot_color_g = data.markers[0].color.g
    robot_color_b = data.markers[0].color.b

def create_ground_truth_data():
    rospy.init_node('ground_truth_data')
    #time.sleep(2) # wait for the obstacles to be spawned
    #rospy.Subscriber('/flatland_server/debug/model/table1', MarkerArray, callback)
    rospy.Subscriber('/flatland_server/debug/topics', DebugTopicList, callback_flatland_server)
    rospy.Subscriber('/flatland_markers', MarkerArray, callback_flatland_markers)
    rospy.Subscriber('/flatland_server/debug/model/myrobot', MarkerArray, callback_myrobot)
    rospy.spin()

if __name__ == '__main__':
    # the calculations and iterations here are moved to a separate node, since they are slowing the data collection (laser scan etc.)
    create_ground_truth_data()
