#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg
import math
from nav_msgs.msg import OccupancyGrid, Odometry
from numpy import asarray
from numpy import savetxt

lp = lg.LaserProjection()
read_laser_scan_info = 1

robot_pos_x = 0.0
robot_pos_y = 0.0
robot_angle_z = 0.0

def calculate_avg_x_value(point_generator):
    sum_temp = 0.0
    num_temp = 0
    for point in point_generator:
        #print(point)
        if not math.isnan(point[0]):
            sum_temp += point[0]
            num_temp += 1
    print('avg x value: ' + str(sum_temp/num_temp)) # calculate avg x value

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
        # Example of ranges - 2 different ones, while the robot was moving:
        #ranges:(4.5, 4.500685214996338, 4.502742767333984, 4.506175518035889, 4.510988235473633, 4.517189025878906, 4.524786949157715, 4.533794403076172, 4.544224262237549, 4.556092739105225, 4.569419860839844, 4.584225177764893, 4.600532531738281, 4.618368625640869, 4.637761116027832, 4.658742904663086, 4.681347370147705, 4.705612659454346, 4.731579780578613, 4.759293079376221, 4.78879976272583, 4.820152759552002, 4.8534064292907715, 4.888621807098389, 4.925863265991211, 4.965200424194336, 5.00670862197876, 5.0504679679870605, 5.096565246582031, 5.1450934410095215, 5.196152210235596, 5.249850273132324, 5.306303024291992, 5.365634441375732, 5.427980899810791, 5.493485450744629, 5.562305927276611, 5.634610176086426, 5.7105817794799805, 5.790417671203613, 5.874332904815674, 5.962558269500732, 6.055346965789795, 6.152973651885986, 6.255735874176025, 6.363961219787598, 6.478003978729248, 6.5982561111450195, 6.725144386291504, 6.8591389656066895, 7.000756740570068, 7.150570869445801, 7.309211254119873, 7.477380275726318, 7.655857563018799, 7.8455095291137695, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, 7.796322345733643, 7.664480686187744, 7.651384353637695, 7.72522497177124, 7.037729263305664, 6.9982075691223145, 7.02544641494751, 7.1818389892578125, 7.510401248931885, 7.512060165405273, 7.098311901092529, 7.039268970489502, 7.051368713378906, 7.154397010803223, 7.7110819816589355, 7.605920314788818, 7.601510047912598, 7.688368797302246, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, 4.070480823516846, 3.8512229919433594, 3.7228245735168457, 3.7459053993225098, 3.3208706378936768, 3.176915407180786, 3.045811891555786, 3.032142400741577, 3.056356430053711, 2.7148077487945557, 2.621406316757202, 2.5349643230438232, 2.4547643661499023, 2.3801867961883545, 2.3565993309020996, 2.3819401264190674, 2.408573865890503, 2.4365601539611816, 2.4659643173217773, 2.496857166290283, 2.529313802719116, 3.1837191581726074, 3.114619255065918, 3.0493643283843994, 2.987678289413452, 2.9293136596679688, 2.8740439414978027, 2.8216652870178223, 2.8284263610839844, 2.879112720489502, 2.9325578212738037, 2.988953113555908, 3.0485055446624756, 3.11144757270813, 3.1780309677124023, 2.703127384185791, 2.667165756225586, 2.6329400539398193, 2.6003634929656982, 2.569356679916382, 2.5398452281951904, 2.5117597579956055, 2.5823323726654053, 2.6599998474121094, 2.743344306945801, 2.8329710960388184, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, 7.845513820648193, 7.655858993530273, 7.477382659912109, 7.309212684631348, 7.150572299957275, 7.000757694244385, 6.859140396118164, 6.725146293640137, 6.598257541656494, 6.478005886077881, 6.363961696624756, 6.2557373046875, 6.152975082397461, 6.055347919464111, 5.962559700012207, 5.874334335327148, 5.790417671203613, 5.710582256317139, 5.634611129760742, 5.562306880950928, 5.493487358093262, 5.427980422973633, 5.365634918212891, 5.306303024291992, 5.249851226806641, 5.19615364074707, 5.1450934410095215, 5.096565246582031, 5.050468444824219, 5.006709575653076, 4.965201377868652, 4.925863265991211, 4.888621807098389, 4.8534064292907715, 4.820152759552002, 4.7888007164001465, 4.759293079376221, 4.7315802574157715, 4.705613136291504, 4.681347846984863, 4.658743381500244, 4.637761116027832, 4.618368625640869, 4.6005330085754395, 4.584225654602051, 4.5694193840026855, 4.556093215942383, 4.544224262237549, 4.533794403076172, 4.524787425994873, 4.517189025878906, 4.510988712310791, 4.506175518035889, 4.502742767333984, 4.500685214996338)
        #ranges:(4.5, 4.500685214996338, 4.502742767333984, 4.506175518035889, 4.510988235473633, 4.517189025878906, 4.524786949157715, 4.533794403076172, 4.544224262237549, 4.556092739105225, 4.569419860839844, 4.584225177764893, 4.600532531738281, 4.618368625640869, 4.637761116027832, 4.658742904663086, 4.681347370147705, 4.705612659454346, 4.731579780578613, 4.759293079376221, 4.78879976272583, 4.820152759552002, 4.8534064292907715, 4.888621807098389, 4.925863265991211, 4.965200424194336, 5.00670862197876, 5.0504679679870605, 5.096565246582031, 5.1450934410095215, 5.196152210235596, 5.249850273132324, 5.306303024291992, 5.365634441375732, 5.427980899810791, 5.493485450744629, 5.562305927276611, 5.634610176086426, 5.7105817794799805, 5.790417671203613, 5.874332904815674, 5.962558269500732, 6.055346965789795, 6.152973651885986, 6.255735874176025, 6.363961219787598, 6.478003978729248, 6.5982561111450195, 6.725144386291504, 6.8591389656066895, 7.000756740570068, 7.150570869445801, 7.309211254119873, 7.477380275726318, 7.655857563018799, 7.8455095291137695, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, 6.402930736541748, 6.3159918785095215, 6.3038763999938965, 6.344244003295898, 5.822469234466553, 5.719971179962158, 5.684741020202637, 5.688231945037842, 5.754312038421631, 6.227901935577393, 6.224567413330078, 6.240228176116943, 5.87115478515625, 5.790133476257324, 5.788827896118164, 5.821356296539307, 5.941709518432617, 6.450442314147949, 6.401341915130615, 6.410106182098389, 6.48675537109375, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, 4.631899833679199, 4.515746116638184, 4.406584739685059, 4.4075164794921875, 4.466944217681885, 4.115750312805176, 4.029520034790039, 3.9480068683624268, 3.870882034301758, 3.797846555709839, 3.850752592086792, 3.917370557785034, 3.9875710010528564, 3.5415494441986084, 3.4853568077087402, 3.4319498538970947, 3.3811678886413574, 3.3328685760498047, 3.286916494369507, 3.3565125465393066, 3.436628580093384, 3.5217623710632324, 4.053043365478516, 4.006490230560303, 3.9621872901916504, 3.920034646987915, 3.999999523162842, 4.125329494476318, 4.260107040405273, 4.4053778648376465, 4.562342166900635, 3.895062208175659, 3.864203453063965, 3.8349881172180176, 3.8073627948760986, 3.781276226043701, 3.8886592388153076, 4.0851640701293945, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, 7.845513820648193, 7.655858993530273, 7.477382659912109, 7.309212684631348, 7.150572299957275, 7.000757694244385, 6.859140396118164, 6.725146293640137, 6.598257541656494, 6.478005886077881, 6.363961696624756, 6.2557373046875, 6.152975082397461, 6.055347919464111, 5.962559700012207, 5.874334335327148, 5.790417671203613, 5.710582256317139, 5.634611129760742, 5.562306880950928, 5.493487358093262, 5.427980422973633, 5.365634918212891, 5.306303024291992, 5.249851226806641, 5.19615364074707, 5.1450934410095215, 5.096565246582031, 5.050468444824219, 5.006709575653076, 4.965201377868652, 4.925863265991211, 4.888621807098389, 4.8534064292907715, 4.820152759552002, 4.7888007164001465, 4.759293079376221, 4.7315802574157715, 4.705613136291504, 4.681347846984863, 4.658743381500244, 4.637761116027832, 4.618368625640869, 4.6005330085754395, 4.584225654602051, 4.5694193840026855, 4.556093215942383, 4.544224262237549, 4.533794403076172, 4.524787425994873, 4.517189025878906, 4.510988712310791, 4.506175518035889, 4.502742767333984, 4.500685214996338)
        read_laser_scan_info = 0

    # convert laser scan data to a point cloud to get a set of 3D Cartesian (x,y,z) points:
    pc2_msg = lp.projectLaser(data)
    point_generator = pc2.read_points(pc2_msg) # a generator of the individual points (x,y,z,index) (could be accessed in a loop)
    point_list = pc2.read_points_list(pc2_msg) # a list of the individual points (x,y,z,index) # example: Point(x=-0.07854820042848587, y=-4.499999523162842, z=0.0, index=359)
    
    # do sth with the converted data:
    #calculate_avg_x_value(point_generator)
    #print(len(point_list))
    #print(point_list[len(point_list)/2].x) # access the x coordinate of a the middle point of the list
    print(point_list[len(point_list)/2]) # access the information about the middle point of the list
    # TODO Question: are the points absolute or relative to the position of the robot - relative?

    print('robot current absolute pos(x,y): ' + str(robot_pos_x) + ',' + str(robot_pos_y))
    print('robot current angle z: ' + str(robot_angle_z))
    # TODO: from the absolute robot coordinates could the relative point cloud data be transformed to an absolute data

    # TODO: save the data (into a rosbag, then to a csv file (preprocessing)!?) and work with it (add semantics, transform to a 2d bird eye view map etc.)
    # TODO: scale up the size of the chairs and tables vs. scale down the robot size
    # TODO: save also the absolute and relative position of the robot the whole time, to be able to match it with the laser scan data; map daten auch um zu wissen wo die obstacles sind
    # TODO: postprocessing: laser scan data to semantic laser scan data (to know that this was a tish for example)
    # TODO: use Daniel's GUI, because later we will need 1000 scenarios

def callback_map(map_data):
    # TODO: a ground truth map with occupied, not occupied, unknown is needed; why does not the map update?
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
    print("FREE: " + str(free_amount) + ", UNKNOWN: " + str(unknown_amount) + ", OCCUPIED: " + str(ocupied_amount)) # FREE: 278156, UNKNOWN: 2134, OCCUPIED: 66696

def callback_odom(odom_data):
    #print('(pos_x, pos_y, angle_z) = (' + str(odom_data.pose.pose.position.x) + ', ' + str(odom_data.pose.pose.position.y) + ', ' + str(odom_data.pose.pose.orientation.z) + ')')
    # TODO: map the position of the robot with its laser scan data
    global robot_pos_x
    global robot_pos_y
    global robot_angle_z
    robot_pos_x = odom_data.pose.pose.position.x
    robot_pos_y = odom_data.pose.pose.position.y
    robot_angle_z = odom_data.pose.pose.orientation.z

def laser_scan_data_listener():
    rospy.init_node('scan_values')
    rospy.Subscriber("/scan", LaserScan, callback) # queue_size=1
    rospy.Subscriber('/map', OccupancyGrid, callback_map) # /move_base/global_costmap/costmap similar to /map
    rospy.Subscriber('/odom', Odometry, callback_odom) # /odom returns how the robot is moving and where is it right now
    rospy.spin()

if __name__ == '__main__':
    laser_scan_data_listener()
