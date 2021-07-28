#!/usr/bin/env python
#
# inputs - gps, wheel_odom, imu
# outputs - motor_commands (init only), odometry
#
#
# init - drive back then forward and calculate yaw.
#

import rospy
import tf
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
import collections
import time

def wrapTo2Pi(a):
  return a%(np.pi*2.0)

class WhereAmINode:
  def __init__(self):
    # Inputs 
    self.gps_sub = rospy.Subscriber('/gps/rtkfix', Odometry, self.gps_callback)
    self.wheel_odom_sub = rospy.Subscriber('/pitranger/out/wheel_odom', Odometry, self.wheel_odom_callback)

    # Outputs
    self.odom_pub = rospy.Publisher('/whereami/odom', Odometry, queue_size=10)
    self.wheel_cmd_pub = rospy.Publisher('/teleop/out/twist_cmd', Twist, queue_size=10)

    # Data structures
    self.gps_fifo = collections.deque(maxlen=100)
    self.wheel_odom_fifo = collections.deque(maxlen=100)
    self.gyro_bias = 0.0
    
    # Drive backward then forward and compute your yaw.
    self.pos, self.yaw = self.active_init()
    print("INITIAL POSE: ({}, {}) {}".format(self.pos[0], self.pos[1], self.yaw*180./np.pi))

    # Broadcast the map -> base_link transform.
    self.br = tf.TransformBroadcaster()
    self.broadcast_transform()

    # More inputs
    self.imu_sub = rospy.Subscriber('/xsens/data', Imu, self.imu_callback)

  def active_init(self):
    # Drive backwards.
    self._drive_then_stop(-0.10, 5.0)
    # Find out where you are.
    start_pos = self._sit_still_and_find_yourself()
    # Drive forwards.
    self._drive_then_stop(0.10, 5.0)
    # Find out where you are now.
    stop_pos = self._sit_still_and_find_yourself()
    # Compute initial yaw.
    dpos = stop_pos - start_pos
    yaw = np.arctan2(dpos[1], dpos[0])
    return stop_pos, yaw

  def _drive_then_stop(self, speed, duration):
    # Drive for duration time.
    msg = Twist()
    msg.linear.x = speed

    for i in range(int(duration/0.1)):
      self.wheel_cmd_pub.publish(msg)
      time.sleep(0.1)

    # Stop driving.
    msg.linear.x = 0.0
    for i in range(5):
      self.wheel_cmd_pub.publish(msg)
      time.sleep(0.1)

  def _sit_still_and_find_yourself(self):
    self.gps_fifo.clear()
    while len(self.gps_fifo) < 100:
      print(len(self.gps_fifo))
      time.sleep(0.1)
    positions = np.array(self.gps_fifo)
    return np.mean(positions, axis=0)

  def gps_callback(self, msg):
    tmp = msg.pose.pose.position
    pos = np.array([tmp.x, tmp.y])
    self.gps_fifo.append(pos)

  def imu_callback(self, msg):
    # If we are moving, update the state estimate.
    self.yaw += 0.01* (msg.angular_velocity.z - self.gyro_bias)
    self.yaw = wrapTo2Pi(self.yaw) 
    self.pos = np.mean(np.array(self.gps_fifo)[-5:], axis=0)
    #print("POSE: ({}, {}) {}".format(self.pos[0], self.pos[1], self.yaw*180./np.pi))

    # Send the robot pose as an odometry message.
    odom_msg = Odometry()
    odom_msg.header = msg.header
    odom_msg.header.frame_id = "map"
    odom_msg.pose.pose.position.x = self.pos[0]
    odom_msg.pose.pose.position.y = self.pos[1]
    odom_msg.pose.pose.position.z = 0.0

    q = quaternion_from_euler(0.0, 0.0, self.yaw, 'sxyz')
    odom_msg.pose.pose.orientation.x = q[0]
    odom_msg.pose.pose.orientation.y = q[1]
    odom_msg.pose.pose.orientation.z = q[2]
    odom_msg.pose.pose.orientation.w = q[3]
    self.odom_pub.publish(odom_msg)

    if abs(self.wheel_odom_fifo[-1][0]) < 0.001 and abs(self.wheel_odom_fifo[-1][1]) < 0.001:
      # If we aren't moving, estimate gyro bias.
      self.gyro_bias = self.gyro_bias*0.999 + msg.angular_velocity.z*0.001

    # Broadcast the latest transform.
    self.broadcast_transform()

  def wheel_odom_callback(self, msg):
    lin = msg.twist.twist.linear.x
    ang = msg.twist.twist.angular.z
    self.wheel_odom_fifo.append([lin, ang])

  def broadcast_transform(self):
    self.br.sendTransform((self.pos[0], self.pos[1], 0),
                           quaternion_from_euler(0, 0, self.yaw),
                           rospy.Time.now(),
                           "base_link",
                           "map")

if __name__=="__main__":
  rospy.init_node('whereami')

  node = WhereAmINode()

  rospy.spin()
