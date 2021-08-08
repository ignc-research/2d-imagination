#!/usr/bin/env python
import rospy
import os
from visualization_msgs.msg import Marker, MarkerArray
from collections import namedtuple

amount_obstacles = 0
read_markers_info = 1
robot_cur_pos_x = robot_cur_pos_y = robot_cur_pos_z = 0
robot_color_r = robot_color_g = robot_color_b = 0

id_type_color_ar = [] # [IDTypeColorRef(id=7, type='chair1', color=r: 0.3 g: 0.0 b: 0.1 a: 0.5), ...]
IDTypeColorRef = namedtuple("IDTypeColorRef", "id type color")
id_type_color_str = ''

def get_obstacles_ref_table():
    my_file = 'table.txt'
    lines = []
    global id_type_color_ar, id_type_color_str
    if not (os.path.exists(my_file) and os.path.getsize(my_file) > 0):
        print('ERROR: File ' + my_file + ' does not exist or is empty!')
        os._exit(0)
    with open('table.txt') as file:
        lines = file.readlines()
    # parse the contents into a useful format
    count = 0
    for line in lines:
        count += 1
        if count > 1: # whout the first line = header
            id = int(line.split('\t')[1])
            type = line.split('\t')[2]
            color = line.split('\t')[4].split('(')[1].split(')')[0]
            RGB_color = (float(color.split(',')[0]), float(color.split(',')[1]), float(color.split(',')[2]))
            id_type_color_ar.append(IDTypeColorRef(id, type, RGB_color))

    # print for debugging
    id_type_color_str = '\tid\tobstacle_type\tRGB_color'
    for elem in id_type_color_ar:
        id_type_color_str += '\n\t' + str(elem.id) + '\t' + str(elem.type) + '\t\t(' + str(elem.color[0]) + ',' + str(elem.color[1]) + ',' + str(elem.color[2]) + ')'
    separator = '/**************************************************************/'
    print(separator + '\n' + id_type_color_str + '\n' + separator)

def fill_markers_rviz(new_marker, marker, scale, obstacle_id, count):
    new_marker.header.frame_id = "map"
    new_marker.header.stamp = rospy.Time.now()
    new_marker.ns = "marker"
    #new_marker.id = obstacle_id # Important: write on top of just one individul of each type
    new_marker.id = count # Important: write on top of all individuals of each type
    new_marker.type = Marker.TEXT_VIEW_FACING
    new_marker.action = Marker.MODIFY
    new_marker.pose.position.x = marker.pose.position.x
    new_marker.pose.position.y = marker.pose.position.y
    new_marker.pose.position.z = 0
    new_marker.pose.orientation.x = 0.0
    new_marker.pose.orientation.y = 0.0
    new_marker.pose.orientation.z = 0.0
    new_marker.pose.orientation.w = 1.0
    new_marker.scale.x = 0.0
    new_marker.scale.y = 0.0
    new_marker.scale.z = scale
    new_marker.color.a = 1.0 # the color of the text should be black
    new_marker.color.r = 0.0
    new_marker.color.g = 0.0
    new_marker.color.b = 0.0
    new_marker.points = []
    new_marker.text = str(obstacle_id)
    return new_marker

def fill_markers_table(marker_table, marker, obstacle_id, obstacle_type):
    marker_table.header.frame_id = "map"
    marker_table.header.stamp = rospy.Time.now()
    marker_table.ns = "marker"
    marker_table.id = obstacle_id # overwrite the id > 0
    marker_table.type = marker.type
    marker_table.action = marker.action
    marker_table.pose = marker.pose # just an example individual of the type
    marker_table.scale = marker.scale
    marker_table.color = marker.color
    marker_table.points = marker.points # just an example individual of the type
    marker_table.text = obstacle_type # write the type here, since marker_table.type should be an integer
    return marker_table

def callback_flatland_markers(markers_data):
    global amount_obstacles, read_markers_info
    amount_obstacles = len(markers_data.markers)
    node_obstacles_amount = rospy.get_param('~obstacles_amount') # roslaunch arena_bringup pedsim_test.launch obstacles_amount:=142
    if amount_obstacles == node_obstacles_amount: # should be publishing the whole time from the point all obstacles have been loaded
        new_marker_array = MarkerArray()
        marker_array_table = MarkerArray()
        count = 0
        origin_x = -6
        origin_y = -6
        scale = 0.7
        obstacle_markers = []
        ObstacleMarker = namedtuple("ObstacleMarker", "x y markers") # a struct per obstacle (center_x, center_y, markers_array)
        text_count = 0
        if read_markers_info == 1: # parse the file only once
            get_obstacles_ref_table()
            read_markers_info = 0
        for marker in markers_data.markers:
            # filter out the robot and the walls & print the text only once per obstacle (so not per obstacle part, otherwise each time the text will be written on top and will look bold at the end)
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
                if not(marker.color.r == robot_color_r and marker.color.g == robot_color_g and marker.color.b == robot_color_b): # filter out the robot
                    if not(center_x == origin_x and center_y == origin_y): # filter out the walls
                        text_count += 1
                        obstacle_id = 0 # 0 is reserved for 'free', so make this the default value, if the obstacle id can not be found
                        obstacle_type = ''
                        count += 1

                        # print the type/id of the obstacle based on the obstacle's color (info from the color-id corespondance table)
                        for elem in id_type_color_ar:
                            #print(str(marker.color.r) + ' ' + str(elem.color[0]) + ' ' + str(marker.color.g) + ' ' + str(elem.color[1]) + ' ' + str(marker.color.b) + ' ' + str(elem.color[2])) # for debugging
                            #print(str(marker.color.r == elem.color[0]) + ' ' + str(marker.color.g == elem.color[1]) + ' ' + str(marker.color.b == elem.color[2])) # for debugging
                            # it gives false when the float is with a lot of numbers after the comma, it can not compare it then correcty
                            if ((round(marker.color.r, 2) == round(elem.color[0], 2)) and (round(marker.color.g, 2) == round(elem.color[1], 2)) and (round(marker.color.b, 2) == round(elem.color[2], 2))):
                                obstacle_id = int(elem.id)
                                obstacle_type = str(elem.type)
                                break
                        
                        # write the obstacle id on top of just one vs. on all individuals of each type:
                        new_marker = Marker()
                        new_marker = fill_markers_rviz(new_marker, marker, scale, obstacle_id, count)
                        new_marker_array.markers.append(new_marker)

                        # Important: a marker pro type!
                        elem_unique = 1
                        for elem in marker_array_table.markers:
                            if elem.id == obstacle_id:
                                elem_unique = 0
                                break
                        if elem_unique == 1:
                            marker_table = Marker()
                            marker_table = fill_markers_table(marker_table, marker, obstacle_id, obstacle_type)
                            marker_array_table.markers.append(marker_table)

                        part_markers = [marker]
                        obstacle_markers.append(ObstacleMarker(center_x, center_y, part_markers))

        #print(len(new_marker_array.markers)) # should be = # obstacles (every obstacle from every type) # 26
        #print(len(marker_array_table.markers)) # should be = # obstacle types (one obstacle per type) # 7

        # publish to a topic, to which rviz should subscribe to vizualise the text (so edit also pedsim_test.rviz):
        marker_pub = rospy.Publisher('/obstacle_labels', MarkerArray, queue_size=10)
        marker_pub.publish(new_marker_array)
        #print('All ' + str(text_count) + ' obstacles have been labeled!') # should be 26

        # publish the correspondence table to a ros topic to be always available without having to parse a txt file each time
        # -> but publish the self-def type IDTypeColorRef is not that trivial
        # -> so convert it to an useful type like MarkerArray
        obstacles_id_type_color_ref_pub = rospy.Publisher('/obstacles_id_type_color_ref', MarkerArray, queue_size=10)
        obstacles_id_type_color_ref_pub.publish(marker_array_table)

def callback_myrobot(data):
    global robot_cur_pos_x, robot_cur_pos_y, robot_cur_pos_z, robot_color_r, robot_color_g, robot_color_b
    robot_cur_pos_x = data.markers[0].pose.position.x
    robot_cur_pos_y = data.markers[0].pose.position.y
    robot_cur_pos_z = data.markers[0].pose.position.z
    robot_color_r = data.markers[0].color.r
    robot_color_g = data.markers[0].color.g
    robot_color_b = data.markers[0].color.b

def show_labels():
    rospy.init_node('show_obstacle_types')
    rospy.Subscriber('/flatland_markers', MarkerArray, callback_flatland_markers)
    rospy.Subscriber('/flatland_server/debug/model/myrobot', MarkerArray, callback_myrobot)
    rospy.spin()
    # TODO: read from file and publish to a ros topic also another important data like the ground truth data (map/array) (since the file create_ground_truth_map.py should be run only once at the beginning for collecting data)

if __name__ == '__main__':
    show_labels()
