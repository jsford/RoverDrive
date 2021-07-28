#!/usr/bin/env python

import rospy
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import collections
import math


def wrapToPi(a):
  a = a % (2*np.pi)
  if a > np.pi:
    a -= 2.0*np.pi
  return a

def angle_between(x, y):
    a = (x - y) % (2*np.pi)
    b = (y - x) % (2*np.pi)
    return -a if a < b else b

class YawnNode:
  def __init__(self, gps_window_size=30, gps_radius=0.15):
    self.gps_window_size = gps_window_size
    self.gps_radius = gps_radius

    self.yaw_error = 0.0
    self.yaw_filter_rate = 0.95

    # NOTE(Jordan): GPS and wheel odom must be published at identical rates.
    self.gps_position_fifo = collections.deque(maxlen=self.gps_window_size)
    self.wheel_odom_fifo = collections.deque(maxlen=self.gps_window_size)

    self.gps_sub = rospy.Subscriber('/gps/rtkfix', Odometry, self.gps_callback)
    self.imu_sub = rospy.Subscriber('/xsens/data', Imu, self.imu_callback)
    self.wheel_odom_sub = rospy.Subscriber('/pitranger/out/wheel_odom', Odometry, self.wheel_odom_callback)

    self.imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=10)

  def wheel_odom_callback(self, msg):
    self.wheel_odom = msg
    self.wheel_odom_fifo.append(msg.twist.twist.linear.x)

  def gps_callback(self, msg):
    tmp = msg.pose.pose.position
    position = np.array([tmp.x, tmp.y, tmp.z])
    self.gps_position_fifo.append(position)

  def imu_callback(self, msg):
    if len(self.gps_position_fifo) < self.gps_window_size:
      return

    if len(self.wheel_odom_fifo) < self.gps_window_size:
      return

    ori = msg.orientation
    imu_roll, imu_pitch, imu_yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])

    # Find the vector from the start of the fifo to the end.
    p0 = self.gps_position_fifo[0]
    p1 = self.gps_position_fifo[-1]
    dpos = p1-p0

    # Flip the GPS yaw to match the IMU orientation
    dpos *= -1

    # If GPS is available, combine it into the current yaw estimate.
    # Don't use GPS if robot recently changed directions
    w0 = self.wheel_odom_fifo[0]
    w1 = self.wheel_odom_fifo[-1]
    if np.linalg.norm(dpos) > self.gps_radius and np.sign(w0) == np.sign(w1):

      # If the rover is driving backward, flip the heading.
      if self.wheel_odom.twist.twist.linear.x < 0:
        dpos *= -1
      gps_yaw = np.arctan2(dpos[1], dpos[0])

      q = quaternion_from_euler(imu_roll, imu_pitch, gps_yaw, 'sxyz')
      msg.orientation.x = q[0]
      msg.orientation.y = q[1]
      msg.orientation.z = q[2]
      msg.orientation.w = q[3]
      self.imu_pub.publish(msg)


if __name__=="__main__":
    rospy.init_node('yawn')

    node = YawnNode()
    rospy.spin()
