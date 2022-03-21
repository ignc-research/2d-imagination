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

def callback(laser_data):
    print('Laser scan info:\n')
    print('angle_min:' + str(laser_data.angle_min) + '\n') # start angle of the scan [rad] # -1.57079637051 rad = -90 deg
    print('angle_max:' + str(laser_data.angle_max) + '\n') # end angle of the scan [rad] # 4.69493579865 rad = +270 deg
    print('angle_increment:' + str(laser_data.angle_increment) + '\n') # angular distance between measurements [rad] # 0.0174532923847 rad = 1 deg
    print('time_increment:' + str(laser_data.time_increment) + '\n') # time between measurements [sec] # 0.0
    print('scan_time:' + str(laser_data.scan_time) + '\n') # time between scans [sec] # 0.0
    print('range_min:' + str(laser_data.range_min) + '\n') # minimum range value [m] # 0.0
    print('range_max:' + str(laser_data.range_max) + '\n') # maximum range value [m] # 8.0
    print('ranges:' + str(laser_data.ranges) + '\n') # range data [m] # a huge array of values # len(data.ranges) = 360 # element = nan, when too slose or too far away
    print('intensities:' + str(laser_data.intensities) + '\n') # intensity data 

def main():
    rospy.init_node('scan_values_simple')
    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    main()