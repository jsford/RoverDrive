#!/usr/bin/env python

import rospy
import numpy as np
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

current_pose = None

NORTH = np.pi/2.0
EAST  = 0.0
SOUTH = -np.pi/2.0
WEST  = np.pi


class Pose:
  def __init__(self, x, y, h):
    self.xy = np.array([x,y])
    self.h = h
  def __repr__(self):
    return "<({:0.3f},{:0.3f}), {:0.3f}>".format(self.x, self.y, self.h)
  @property
  def x(self):
    return self.xy[0]
  @property
  def y(self):
    return self.xy[1]

def odometry_callback(msg):
  global current_pose

  position = msg.pose.pose.position
  orientation = msg.pose.pose.orientation

  ori = [orientation.x, orientation.y, orientation.z, orientation.w]
  roll, pitch, yaw = euler_from_quaternion(ori)

  current_pose = Pose(position.x, position.y, yaw)
  
def subtract_angles(x,y):
  a = (x-y) % (2*np.pi)
  b = (y-x) % (2*np.pi)
  return -a if a < b else b

def deg2rad(d):
  return d*np.pi/180.0

def rad2deg(r):
  return r*180.0/np.pi

def get_twist(current_pose, goal_pose):
  vec_to_goal = goal_pose.xy-current_pose.xy
  angle_to_goal = np.arctan2(vec_to_goal[1], vec_to_goal[0])
  dist_to_goal = np.linalg.norm(vec_to_goal)

  # Compute the angle between the robot heading and the vector to the goal.
  theta = subtract_angles(current_pose.h, angle_to_goal)

  # Compute the angle between the robot heading and the goal heading.
  phi = subtract_angles(current_pose.h, goal_pose.h)

  print("DIST: {} THETA: {} PHI: {}\n".format(dist_to_goal, theta, phi))

  if dist_to_goal < 0.5:
    if abs(phi) > deg2rad(10):
      # Reorient
      print("REORIENT\n")
      return [0.0, 0.1*np.sign(phi)], False
    else:
      # Done!
      print("DONE\n")
      return [0.0, 0.0], True
  elif abs(theta) > deg2rad(10):
    # Orient
    print("ORIENT\n")
    return [0.0, 0.1*np.sign(theta)], False
  else:
    # Drive an arc.
    ROBOT_VELOCITY = 0.08
    R = dist_to_goal / (2*np.sin(theta))
    print("DRIVING\n")
    return [ROBOT_VELOCITY, ROBOT_VELOCITY / R], False

  return [0.0, 0.0]

if __name__=="__main__":
  rospy.init_node('goto')

  rate = rospy.Rate(30)

  odom_sub = rospy.Subscriber('odometry/filtered/global', Odometry, odometry_callback)
  twist_pub = rospy.Publisher('/teleop/out/twist_cmd', Twist, queue_size=10)

  waypoints = [
    Pose(0.0, 0.0, NORTH),
    Pose(0.0, 2.0, WEST),
    Pose(-2.0, 2.0, SOUTH),
    Pose(-2.0, 0.0, EAST),
    Pose(0.0, 0.0, EAST),
  ]

  waypoint_idx = 0
  while not rospy.is_shutdown() and waypoint_idx < len(waypoints):

    if current_pose is not None:
      goal_pose = waypoints[waypoint_idx]

      # Otherwise, compute twist command to move toward the goal.
      twist, made_it = get_twist(current_pose, goal_pose)
      print(current_pose, goal_pose, twist)

      if made_it:
        waypoint_idx += 1

      if waypoint_idx == len(waypoints):
        twist = np.array([0.0, 0.0])

      msg = Twist()
      msg.linear.x = twist[0]
      msg.linear.y = 0.0
      msg.linear.z = 0.0
      msg.angular.x = 0.0
      msg.angular.y = 0.0
      msg.angular.z = twist[1]
      twist_pub.publish(msg)

    rate.sleep()





