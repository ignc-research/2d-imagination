#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

def show_goal_publisher():
    vel_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped)
    rospy.init_node('show_goal', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pose_stamped = PoseStamped()
        # move the robot to an example goal position:
        # rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 13.8938770294, y: 8.7637386322, z: 0.0}, orientation: {w: 1.0}}}'
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose.position.x = 13.8938770294
        pose_stamped.pose.position.y = 8.7637386322
        pose_stamped.pose.position.z = 0.0
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = 0.0
        pose_stamped.pose.orientation.w = 1.0
        vel_pub.publish(pose_stamped)
        rate.sleep()

if __name__ == '__main__':
    show_goal_publisher()
