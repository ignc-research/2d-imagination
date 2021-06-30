#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion # to make it work with python3 do: cd $HOME/geometry2_ws $ source devel/setup.bash $ cd $HOME/catkin_ws_ma $ source devel/setup.bash
from geometry_msgs.msg import Point, Twist
from math import atan2
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus, GoalStatusArray

x_global = 0.0
y_global = 0.0 
theta_global = 0.0

def newOdom(msg):
    global x_global
    global y_global
    global theta_global
    x_global = msg.pose.pose.position.x
    y_global = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta_global) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def goal_publisher():
    rospy.init_node("reach_goal", anonymous=True)
    sub = rospy.Subscriber("/odom", Odometry, newOdom) # /odom, /odometry/ground_truth are on the topics list # "/odometry/filtered" does not exist
    pub = rospy.Publisher("/cmd_vel", Twist)
    speed = Twist()
    rate = rospy.Rate(10)
    goal = Point()
    goal.x = 13.8938770294
    goal.y = 8.7637386322
    while not rospy.is_shutdown():
        inc_x = goal.x -x_global
        inc_y = goal.y -y_global
        angle_to_goal = atan2(inc_y, inc_x)
        if abs(angle_to_goal - theta_global) > 0.1:
            speed.linear.x = 0.0
            speed.angular.z = 0.3
        else:
            speed.linear.x = 0.5
            speed.angular.z = 0.0
        pub.publish(speed)
        rate.sleep()

def goal_publisher_move_base():
    rospy.init_node("reach_goal", anonymous=True)
    pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        goal = MoveBaseActionGoal()
        goal.header.seq = 0
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = ""
        goal.goal_id.stamp.secs = 0
        goal.goal_id.stamp.nsecs = 0
        goal.goal_id.id = ""
        goal.goal.target_pose.header.seq = 0
        goal.goal.target_pose.header.stamp = rospy.Time.now()
        goal.goal.target_pose.header.frame_id = "map"
        goal.goal.target_pose.pose.position.x = 13.8938770294
        goal.goal.target_pose.pose.position.y = 8.7637386322
        goal.goal.target_pose.pose.position.z = 0.0
        goal.goal.target_pose.pose.orientation.x = 0.0
        goal.goal.target_pose.pose.orientation.y = 0.0
        goal.goal.target_pose.pose.orientation.z = 0.0
        goal.goal.target_pose.pose.orientation.w = 1.0
        pub.publish(goal)
        rate.sleep()

def goal_status_sub_callback(msg): # check if the goal has been reached or not
    for el in msg.status_list:
        if el.status == 3:
            print("GOAL REACHED!") # TODO: should be printed once per reached goal
            break

def movebase_client(x, y, goal_pub):
    goal = MoveBaseGoal()
    goal.target_pose.header.seq = 0
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = 1.0

    pose_stamped = PoseStamped()
    pose_stamped = goal.target_pose
    goal_pub.publish(pose_stamped)

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    client.send_goal(goal)
    wait = client.wait_for_result()
    if wait:
        return client.get_result()
    #else:
    #    rospy.logerr("Action server not available!")
    #    rospy.signal_shutdown("Action server not available!")

def goal_publisher_move_base_client():
    rospy.init_node('reach_goal', anonymous=True)
    goal_status_sub = rospy.Subscriber("/move_base/status", GoalStatusArray, goal_status_sub_callback)
    goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped) # TODO: shows the goals (their position and orientation with an arrow), but starting with the second goal?
    # TODO: control the speed, overwrite only the linear.x value of the /cmd_vel topic!?

    # move directly to an example position (for debugging purposes)
    #movebase_client(5, 0, goal_pub) # move just slightly
    #movebase_client(13.8938770294, 4.7637386322, goal_pub) # go in between the tables

    # move zig-zag from left to right
    # the robot automatically waits until the current goal has been reached before pursuing the next one
    x_min = -3
    x_max = 25
    y_min = 0
    y_max = 17.5
    y_step = 2.5
    y_temp = y_min
    while y_temp <= y_max - y_step:
        movebase_client(x_max, y_temp, goal_pub)
        y_temp += y_step
        movebase_client(x_min, y_temp, goal_pub)

if __name__ == '__main__':
    #goal_publisher() # the robot moves to the goal, but does not avoid obstacles
    #goal_publisher_move_base() # the robot doeas not move (theoretically it should avoid the obstacles)
    goal_publisher_move_base_client() # the robot moves to goal and avoids the obstacles
