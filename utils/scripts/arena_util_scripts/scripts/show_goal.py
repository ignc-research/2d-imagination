#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

def show_goal_publisher(): # the position and orientation of the robot is shown by an arrow, but the robot does not go there
    goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped)
    rospy.init_node('show_goal', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pose_stamped = PoseStamped()
        # move the robot to an example goal position: (it moves from the cmd line (directly overwrites the current goal), but not from here!?)
        # rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 13.8938770294, y: 8.7637386322, z: 0.0}, orientation: {w: 1.0}}}'
        pose_stamped.header.seq = 0
        pose_stamped.header.stamp = rospy.Time.now()
        #pose_stamped.header.stamp.secs = 0
        #pose_stamped.header.stamp.nsecs = 0
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose.position.x = 13.8938770294
        pose_stamped.pose.position.y = 8.7637386322
        pose_stamped.pose.position.z = 0.0
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = 0.0
        pose_stamped.pose.orientation.w = 1.0
        goal_pub.publish(pose_stamped)
        rate.sleep()

    # TODO: show the goal that is already given to another topic from the reach_goal node for example!?

if __name__ == '__main__':
    show_goal_publisher()
