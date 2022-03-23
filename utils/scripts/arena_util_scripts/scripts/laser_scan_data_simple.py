import rospy
from sensor_msgs.msg import LaserScan

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