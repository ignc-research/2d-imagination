#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

vel_pub = rospy.Publisher("/cmd_vel", Twist)

def zig_zag():
    rate = rospy.Rate(10) # [Hz] = [1/sec]
    for i in range(0, 150):
        turn(0.5, 0.780)
        rate.sleep() # or rospy.sleep(1.0) for sleep for 1 second
        move(0.5)
        rate.sleep()
        turn(0.5, -0.780)
        rate.sleep()
        move(0.5)
        rate.sleep()
    for i in range(0, 150):
        turn(-0.5, -0.780)
        rate.sleep() # or rospy.sleep(1.0) for sleep for 1 second
        move(-0.5)
        rate.sleep()
        turn(-0.5, 0.780)
        rate.sleep()
        move(-0.5)
        rate.sleep()

def move(vel):
    msg = Twist()
    msg.linear.x = vel
    msg.linear.y =  0.0 
    msg.linear.z =  0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0
    vel_pub.publish(msg)

def turn(vel, angle):
    msg = Twist()
    msg.linear.x = vel
    msg.linear.y =  0.0 
    msg.linear.z =  0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = angle
    vel_pub.publish(msg)

def velocity_publisher():
    rospy.init_node('zig_zag', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # Move the robot in the environment in order to take different laser scan data
        #move(0.5) # move forward
        zig_zag() # move zig-zag (multiple zig-zag movements)
        rate.sleep()

if __name__ == '__main__':
    velocity_publisher()
    # INFO: the laser scan data should be taken not only once per whole zig-zag movement,
    # but during the whole movement of the robot, that is why this separate node for only
    # publishing the velocity of the robot was needed
