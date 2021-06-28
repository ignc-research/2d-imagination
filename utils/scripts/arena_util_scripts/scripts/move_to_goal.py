#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion # to make it work with python3 do: cd $HOME/geometry2 $ catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 $source devel/setup.bash
from geometry_msgs.msg import Point, Twist
from math import atan2
from geometry_msgs.msg import PoseStamped

x = 0.0
y = 0.0 
theta = 0.0

def newOdom(msg):
    global x
    global y
    global theta
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def goal_publisher():
    rospy.init_node("reach_goal", anonymous=True)
    sub = rospy.Subscriber("/odom", Odometry, newOdom) # /odom, /odometry/ground_truth are on the list # "/odometry/filtered" do not exist
    pub = rospy.Publisher("/cmd_vel", Twist)
    speed = Twist()
    rate = rospy.Rate(10)
    goal = Point()
    goal.x = 13.8938770294
    goal.y = 8.7637386322
    while not rospy.is_shutdown():
        inc_x = goal.x -x
        inc_y = goal.y -y
        angle_to_goal = atan2(inc_y, inc_x)
        if abs(angle_to_goal - theta) > 0.1:
            speed.linear.x = 0.0
            speed.angular.z = 0.3
        else:
            speed.linear.x = 0.5
            speed.angular.z = 0.0
        pub.publish(speed)
        rate.sleep()

if __name__ == '__main__':
    goal_publisher()
