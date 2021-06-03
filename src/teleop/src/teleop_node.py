#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from controller import Controller

if __name__=="__main__":
    rospy.init_node('teleop')

    controller = Controller()
    twist_pub = rospy.Publisher('/teleop/out/twist_cmd', Twist, queue_size=1)

    max_speed_mps = 0.12
    max_yaw_rps   = 0.02

    iteration = 0
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        controller.update();
        print('hi')

        if iteration%10 == 0:
            left_joy_x = controller.lookup('LEFT_JOY_X')
            left_joy_y = controller.lookup('LEFT_JOY_Y')

            x_axis = (left_joy_x-128)/128.0
            y_axis = (128-left_joy_y)/128.0

            msg = Twist()
            msg.linear.x = y_axis * max_speed_mps
            msg.angular.z = x_axis * max_yaw_rps
            twist_pub.publish(msg)
        iteration += 1
        rate.sleep()
