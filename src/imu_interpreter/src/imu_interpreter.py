#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion


def callback(msg):
    qx = msg.orientation.x;
    qy = msg.orientation.y;
    qz = msg.orientation.z;
    qw = msg.orientation.w;
    roll, pitch, yaw = euler_from_quaternion([qx, qy, qz, qw])
    print("Roll: {:.3f} Pitch: {:.3f} Yaw: {:.3f}".format(roll * 180.0/np.pi, pitch * 180.0/np.pi, yaw * 180.0/np.pi))

if __name__=="__main__":
    rospy.init_node('imu_interpreter')

    sub = rospy.Subscriber('imu/data', Imu, callback)

    rospy.spin()

