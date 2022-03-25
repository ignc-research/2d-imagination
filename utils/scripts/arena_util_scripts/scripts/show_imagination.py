#!/usr/bin/env python
import os
import sys
import rospy
import cv2
import numpy as np
import yaml
import rospkg
import torch
from nav_msgs.msg import Odometry, OccupancyGrid
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan

class node_show_imagination():
    def __init__(self):
        
        rospack = rospkg.RosPack()
        # get the parameters map_resolution, x_max, y_max, x_offset, y_offset directly from the chosen map
        self.map_file = rospy.get_param('~map_file') # "map_empty"
        self.map_path = rospy.get_param('~map_path')
        # parse the .yaml file
        with open(self.map_path, 'r') as stream:
            try:
                self.doc = yaml.safe_load(stream)
                self.map_resolution = float(self.doc['resolution']) # resolution: 0.05
                self.x_offset = float(self.doc['origin'][0]) # origin: [-6.0, -6.0, 0.0]
                self.y_offset = float(self.doc['origin'][1])
                self.image = self.doc['image'] # image: map_small.png
                # read the image and get its dimensions
                self.map_img_path = os.path.join(rospack.get_path("simulator_setup"), "maps", self.map_file, self.image)
                self.map_img = cv2.imread(self.map_img_path)
                self.x_max = self.map_img.shape[0] # 521
                self.y_max = self.map_img.shape[1] # 666
            except yaml.YAMLError as exc:
                print(exc)
        
        self.imagination_global_pub = rospy.Publisher("/imagination_global", OccupancyGrid, queue_size=10) 
        self.counter = 0
        self.LaserData = []

        self.imagination_size = int(rospy.get_param('~imagination_size')) + 1 # 60+1/80+1/100+1
        self.map_extend_len = self.imagination_size - 1
        self.map_extend_len_half = int(self.map_extend_len/2)
        
        self.imagination_global_map_bool= np.zeros((self.x_max+self.map_extend_len, self.y_max + self.map_extend_len))*255
        self.imagination_global_map= np.zeros((self.x_max, self.y_max,3))*255

        # robot pos
        self.robot_x = 0
        self.robot_y = 0
        self.theta = 0

        #gt map
        self.se_gt_map_path = "./map_ground_truth_semantic.png"
        if not(os.path.isfile(self.se_gt_map_path)):
            error_msg = "\nThere is no ground truth data generated! Launch pedsim_test_gt.launch first.\nRun for example $ roslaunch arena_bringup pedsim_test_gt.launch scenario:=1 gt_extension:=0\n"
            #sys.exit(error_msg)
            raise rospy.ROSInternalException(error_msg)
        self.gt_map = cv2.imread(self.se_gt_map_path)
        self.gt_map = cv2.cvtColor(self.gt_map, cv2.COLOR_BGR2RGB)
        self.gt_map = np.around(self.gt_map/255,1)

        # colour reference
        self.colourReference = {
            (0.3,0.0,0.1) : 1,
            (0.0,0.6,0.1) : 2,
            (0.1,0.6,1.0) : 3,
            (0.4,0.0,0.5) : 4,
            (0.5,0.1,0.0) : 5,
            (0.6,1.0,0.3) : 6,
            (0.7,0.0,0.0) : 7,
            (0.7,0.0,0.5) : 8,
            (0.7,0.3,0.6) : 9,
            (0.8,0.5,0.1) : 10
        }
        
        # load model
        self.node_user = rospy.get_param('~user')
        self.node_workspace = rospy.get_param('~workspace')
        self.imagination_path = '/home/' + self.node_user + '/' + self.node_workspace + '/src/rosnav-imagination' # where the repository "https://github.com/ignc-research/rosnav-imagination" has been cloned
        #imagination_path = os.path.join(rospack.get_path("arena_bringup"), "../../rosnav-imagination") # alternative # where the repository "https://github.com/ignc-research/rosnav-imagination" has been cloned
        sys.path.insert(0,self.imagination_path)
        self.current_model_number = rospy.get_param('~imagination_model') # "3000"/"3000_60_normal"/...
        self.model_path = self.imagination_path + "/example/models/model_" + str(self.current_model_number) + ".pth"
        #self.model_path = "./models/60x60-ext/model_3000.pth"
        self.device = rospy.get_param('~device') # 'cpu'/'cuda'/'cuda:0'
        if self.device == 'cpu': self.anticipator = torch.load(self.model_path, map_location=torch.device(self.device)) # CPU
        else: self.anticipator = torch.load(self.model_path).to(self.device) # GPU

    def grey_img_to_grid(self, grey_img):
        map_array=np.ones((self.x_max,self.y_max),dtype=int)*100
        white_ar = (255,255,255)
        black_ar = (0,0,0)

        grey_img_resorted = grey_img[::-1]
        white_index = np.all(grey_img_resorted == white_ar, axis=-1)
        black_index = np.all(grey_img_resorted == black_ar, axis=-1)

        map_array[white_index] = -1
        map_array[black_index] = 0
        map_array_flat = map_array.flatten().tolist()
        return map_array_flat

    def laser_to_img_coord(self,LaserData):
        # in: raw laser data
        # out: (int, int ) array
        N = len(LaserData.ranges)
        ranges = np.array(LaserData.ranges)
        self.__angle_min = LaserData.angle_min
        self.__angle_max = LaserData.angle_max
        angles = LaserData.angle_min + np.arange(N) * LaserData.angle_increment
        self.__cos_sin_map = np.array([np.cos(angles), np.sin(angles)])
        output = ranges * self.__cos_sin_map
        robot_pos = np.array([self.robot_x, self.robot_y])
        output = output.T + robot_pos

        output = self.ros_to_img_coord(output)
        return output

    def laser_to_robot_coord(self,LaserData):
        # in: raw laser data
        # out: (int, int ) array
        N = len(LaserData.ranges)
        ranges = np.array(LaserData.ranges)
        self.__angle_min = LaserData.angle_min
        self.__angle_max = LaserData.angle_max
        angles = LaserData.angle_min + np.arange(N) * LaserData.angle_increment
        self.__cos_sin_map = np.array([np.cos(angles), np.sin(angles)])
        output = ranges * self.__cos_sin_map
        output = output.T
        output = self.ros_to_img_coord(output)
        return output

    def ros_to_img_coord(self, robot_pos):
        robot_pos[:, 1] = -robot_pos[:, 1]
        x_max = self.x_max*self.map_resolution
        y_max = self.y_max*self.map_resolution
        offset = np.array([abs(self.y_offset),x_max - abs(self.x_offset)])
        map_pos = (robot_pos/self.map_resolution + offset/self.map_resolution).astype(int)
        return np.array([map_pos[:,1], map_pos[:,0]]).T

    def color_to_id(self, color):
        t_color = tuple(color)
        if t_color in self.colourReference.keys(): return self.colourReference[t_color]
        else: return 0

    def normalized(self, a):
        vmax = np.max(a)
        vmin = np.min(a)
        if vmax-vmin == 0: return 0
        a = (a - vmin)/(vmax-vmin)
        return a 

    def points_to_semantic_input(self, LaserData):
        scan_2d_points = self.laser_to_img_coord(LaserData)
        indexs = np.all([np.all(scan_2d_points > (0,0), axis=-1), np.all(scan_2d_points < (self.x_max,self.y_max), axis=-1)],axis=0)
        laser_img = np.zeros((self.x_max+self.map_extend_len,self.y_max+self.map_extend_len)) #put more area on the edge for cut

        robot_pos = np.array([self.robot_x, self.robot_y])[np.newaxis]
        robot_pos_map = self.ros_to_img_coord(robot_pos).squeeze()

        for point in scan_2d_points[indexs]:
            # take the 5x5 around points to confirm if the points from an object
            start_index_x = np.max([0,point[0]-2]) 
            start_index_y = np.max([0,point[1]-2])
            end_index_x = np.min([self.x_max,point[0]+3])
            end_index_y = np.min([self.y_max,point[1]+3])

            object_matrix = self.gt_map[start_index_x:end_index_x,start_index_y:end_index_y]
            object_matrix = object_matrix.reshape(-1,object_matrix.shape[-1])

            id_matrix = np.array(list(map(self.color_to_id, object_matrix)))
            laser_img[point[0]+self.map_extend_len_half-2:point[0]+self.map_extend_len_half, point[1]+self.map_extend_len_half-2:point[1]+self.map_extend_len_half] = np.max(id_matrix)

        observation = laser_img[robot_pos_map[0]:robot_pos_map[0]+self.imagination_size, robot_pos_map[1]:robot_pos_map[1] + self.imagination_size] 

        if self.device == 'cpu': observation_device = torch.tensor(observation[np.newaxis,np.newaxis].astype(np.float32)) # CPU
        else: observation_device = torch.tensor(observation[np.newaxis,np.newaxis].astype(np.float32)).to(self.device) # GPU
        result = self.anticipator(observation_device)
        raw_imagination = result['occ_estimate'].to(self.device).detach().numpy()[0,0]
        imagination = self.normalized(raw_imagination)
        dilation_mask = np.ones((6, 6))
        current_mask = cv2.dilate(observation, dilation_mask, iterations=1,).astype(np.float32)
        current_mask = cv2.GaussianBlur(current_mask,(31,31),0)
        imagination = ((current_mask*imagination)>0.1).astype(np.int32)
        
        #plt.imsave('test_imagination.png', imagination)
        #plt.imsave('raw_imagination.png', raw_imagination)
        #plt.imsave('test_observation.png', observation)
        #plt.imsave('test_laser.png', laser_img[self.map_extend_len_half:self.map_extend_len_half+self.x_max,self.map_extend_len_half:self.map_extend_len_half+self.y_max])
        return imagination, robot_pos_map

    def imagination_to_laser_data(self, imagination, LaserData):
        kernel = np.ones((5,5),np.uint8)
        imagination_filter1_threshold = rospy.get_param('~imagination_filter1_threshold') # 0.1/0.2/0.3
        obs = np.array(self.normalized(imagination.astype(np.float32)) > imagination_filter1_threshold).astype(np.float32)
        obs = (cv2.morphologyEx(obs, cv2.MORPH_CLOSE, kernel))
        #plt.imsave('MORPH_CLOSE.png', obs)

        sobely = np.abs(cv2.Sobel(obs,cv2.CV_64F,1,0,ksize=1)) + np.abs(cv2.Sobel(obs,cv2.CV_64F,0,1,ksize=1))
        #plt.imsave('Sobel.png', sobely)

        index = np.array(np.where(sobely > 0)).T 
        index_ros = np.zeros(index.shape)
        index_ros[:,0] = index[:,1]*self.map_resolution - self.map_resolution*self.map_extend_len_half
        index_ros[:,1] = -index[:,0]*self.map_resolution + self.map_resolution*self.map_extend_len_half
        r = np.sqrt(index_ros[:,0]**2+index_ros[:,1]**2)
        t = np.arctan2(index_ros[:,1],index_ros[:,0])

        len_ranges = 1080
        delta_angle = np.pi/len_ranges*2
        ranges = 300*np.ones(len_ranges)

        for i in range(len_ranges):
            angle_min = -np.pi/2 + self.theta + i*delta_angle
            angle_max = -np.pi/2 + self.theta + (i + 1)*delta_angle
            index = np.where(np.all([t > angle_min, t <angle_max],axis=0))
            if  len(index[0]) > 0:
                start_ind = int(max(i-2, 0))
                end_ind = int(min(i+3,len_ranges))
                r_arr = np.min(r[index])*np.ones(ranges[start_ind:end_ind].shape)
                ranges[start_ind:end_ind] = np.min([r_arr,ranges[start_ind:end_ind]],axis=0)

        ranges[::3] = np.array(LaserData.ranges)
        laser_imagination = LaserData
        laser_imagination.ranges = ranges.tolist()
        laser_imagination.header.stamp = rospy.Time.now()
        laser_imagination.angle_increment = delta_angle
        laser_imagination.angle_min = -np.pi/2
        laser_imagination.range_min = 0.1
        laser_imagination.range_max  = 8
        pub_laser_scan_imagination = rospy.Publisher("/imagination_laser_scan", LaserScan, queue_size=10) 
        pub_laser_scan_imagination.publish(laser_imagination)
        
    def update_map_topic(self, grid):
        # Important: since both the local_costmap and the global_costmap use the /map topic as a static_layer, they will also update themselves, just like the local and global planner
        
        pub_map = rospy.Publisher("/map", OccupancyGrid, queue_size=10)
        grid_map_topic = OccupancyGrid()
        grid_map_topic = grid
        grid_map_topic.header.frame_id = "map"

        # necessary to prevent the topic from loosing its initial data
        # TODO: it is enough to be done once (after the init of map_topic.png)
        map_topic_img = cv2.imread("map_topic.png")
        imagination_map_global_img = cv2.imread("imagination_map_global.png") # self.imagination_global_map
        map_topic_img_new = cv2.add(map_topic_img, imagination_map_global_img)
        grid_map_topic.data = self.grey_img_to_grid(map_topic_img_new)

        pub_map.publish(grid_map_topic)
    
    def laser_scan_callback(self, LaserData):
        self.counter +=1
        #print(f'coumter: {self.counter} -Laser scan info received:\n')
        step = 4
        if self.counter%step == 0:
            #print(f'imagination start:\n')
            imagination, robot_pos_map = self.points_to_semantic_input(LaserData)

            imagination_area = self.imagination_global_map_bool[robot_pos_map[0]:robot_pos_map[0]+self.imagination_size, robot_pos_map[1]:robot_pos_map[1]+self.imagination_size]

            imagination_area += imagination
            #np.save('imagination_area.npy',imagination_area)
            #plt.imsave('imagination_area.png', imagination_area)

            self.imagination_global_map_bool = (self.imagination_global_map_bool > 0).astype(np.int32)
            for i in range(self.imagination_global_map.shape[-1]):
                self.imagination_global_map[:,:,i] = 100*self.imagination_global_map_bool[self.map_extend_len_half:self.map_extend_len_half+self.x_max,self.map_extend_len_half:self.map_extend_len_half+self.y_max] 
        
            cv2.imwrite("imagination_map_global.png", self.imagination_global_map)

            grid = OccupancyGrid()
            grid.header.seq = 0
            grid.header.stamp = rospy.Time.now()
            grid.header.frame_id = ""
            # get info directly from the subscribed topic, but change width, hight and position!
            grid.info.width = self.imagination_global_map.shape[1] # 666 !
            grid.info.height = self.imagination_global_map.shape[0] # 521 !
            grid.info.resolution = self.map_resolution
            #grid.info.map_load_time = map_data.info.map_load_time
            grid.info.origin.orientation.x = 0
            grid.info.origin.orientation.y = 0
            grid.info.origin.orientation.z = 0
            grid.info.origin.orientation.w = 1
            grid.info.origin.position.x = self.x_offset # !
            grid.info.origin.position.y = self.y_offset # !
            grid.info.origin.position.z = 0
            grid.data = self.grey_img_to_grid(self.imagination_global_map)
            self.imagination_global_pub.publish(grid)
            self.imagination_to_laser_data(imagination_area, LaserData)

            self.update_map_topic(grid)
            
        '''
        else:
            len_ranges = 1080
            delta_angle = np.pi/len_ranges*2
            ranges = np.ones(len_ranges)*300
            ranges[::3] = np.array(LaserData.ranges)
            laser_imagination = LaserData
            laser_imagination.ranges = ranges.tolist()
            laser_imagination.header.stamp = rospy.Time.now()
            laser_imagination.angle_increment = delta_angle
            laser_imagination.range_min = 0.1
            laser_imagination.range_max  = 8
            #print(f"laser_imagination.angle_min: {laser_imagination.angle_min} \n Robot angle: {self.theta}")
            pub_laser_scan_imagination = rospy.Publisher("/imagination_laser_scan", LaserScan, queue_size=10) 
            pub_laser_scan_imagination.publish(laser_imagination)
        '''

    def odom_callback(self,OdomData):
        self.robot_x = OdomData.pose.pose.position.x
        self.robot_y = OdomData.pose.pose.position.y
        self.rot_q = OdomData.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion([self.rot_q.x, self.rot_q.y, self.rot_q.z, self.rot_q.w])

def main():
    rospy.init_node("show_imagination")

    node = node_show_imagination()
    rospy.Subscriber("/scan", LaserScan, node.laser_scan_callback)
    rospy.Subscriber("/odom", Odometry, node.odom_callback)

    rospy.spin()

if __name__ == '__main__':
    main()