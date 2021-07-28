#!/usr/bin/env python

import rospy
import numpy as np
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

current_pose = None

NORTH = np.pi/2.0
NORTH_EAST = np.pi/4.0
EAST  = 0.0
SOUTH_EAST = -np.pi/4.0
SOUTH = -np.pi/2.0
SOUTH_WEST = -3*np.pi/4.0
WEST  = np.pi
NORTH_WEST = 3*np.pi/4.0
  
def subtract_angles(x,y):
  a = (x-y) % (2*np.pi)
  b = (y-x) % (2*np.pi)
  return -a if a < b else b

def deg2rad(d):
  return d*np.pi/180.0

def rad2deg(r):
  return r*180.0/np.pi


class Pose:
  def __init__(self, x, y, h, dxy=0.5, dh=deg2rad(5)):
    self.xy = np.array([x,y])
    self.h = h
    self.dxy = abs(dxy)
    self.dh = abs(dh)
  def __repr__(self):
    return "<({:0.3f},{:0.3f}), {:0.3f}>".format(self.x, self.y, rad2deg(self.h))
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

def get_twist(current_pose, goal_pose):
  vec_to_goal = goal_pose.xy-current_pose.xy
  angle_to_goal = np.arctan2(vec_to_goal[1], vec_to_goal[0])
  dist_to_goal = np.linalg.norm(vec_to_goal)

  # Compute the angle between the robot heading and the vector to the goal.
  theta = subtract_angles(current_pose.h, angle_to_goal)

  # Compute the angle between the robot heading and the goal heading.
  phi = subtract_angles(current_pose.h, goal_pose.h)

  print("DIST: {} THETA: {} PHI: {} DXY: {} DH: {}\n".format(
        dist_to_goal, rad2deg(theta), rad2deg(phi), goal_pose.dxy, rad2deg(goal_pose.dh)))

  if dist_to_goal <= goal_pose.dxy:
    if abs(phi) > goal_pose.dh:
      # Reorient
      print("REORIENT")
      return [0.0, 0.1*np.sign(phi)], False
    else:
      # Done!
      print("DONE")
      return [0.0, 0.0], True
  elif abs(theta) > deg2rad(5):
    # Orient
    print("ORIENT")
    return [0.0, 0.1*np.sign(theta)], False
  else:
    # Drive an arc.
    ROBOT_VELOCITY = 0.08
    R = dist_to_goal / (2*np.sin(theta))
    print("DRIVING")
    return [ROBOT_VELOCITY, ROBOT_VELOCITY / R], False
  return [0.0, 0.0]

if __name__=="__main__":
  rospy.init_node('goto')

  rate = rospy.Rate(30)

  odom_sub = rospy.Subscriber('/whereami/odom', Odometry, odometry_callback)
  twist_pub = rospy.Publisher('/teleop/out/twist_cmd', Twist, queue_size=10)

  # Drive a square
  square_points = [
    Pose(0.0, 0.0, NORTH),
    Pose(0.0, 5.0, WEST),
    Pose(-5.0, 5.0, SOUTH),
    Pose(-5.0, 0.0, EAST),
    Pose(0.0, 0.0, EAST),
  ]

  # Drive a CMU
  cmu_points = [
    # C
    Pose(6-6, 10-10, NORTH_WEST),
    Pose(4-6, 12-10, WEST),
    Pose(2-6, 12-10, SOUTH_WEST),
    Pose(0-6, 10-10, SOUTH),
    Pose(0-6,  2-10, SOUTH_EAST),
    Pose(2-6,  0-10, EAST),
    Pose(4-6,  0-10, NORTH_EAST),
    Pose(6-6,  2-10, SOUTH_EAST),
    # M
    Pose( 8-6,  0-10, NORTH),
    Pose( 8-6, 12-10, SOUTH_EAST),
    Pose(11-6,  6-10, NORTH_EAST),
    Pose(14-6, 12-10, SOUTH),
    Pose(14-6,  0-10, EAST),
    # U
    Pose(16-6,   0-10, NORTH),
    Pose(16-6,  12-10, SOUTH),
    Pose(16-6,   0-10, WEST),
    Pose(22-6,   0-10, NORTH),
    Pose(22-6,  12-10, NORTH)
  ]

  # Drive a pentagon
  A = deg2rad(72)
  c1 = np.cos(2*np.pi/5.)
  c2 = np.cos(np.pi/5.)
  s1 = np.sin(2*np.pi/5.)
  s2 = np.sin(np.pi/5.)

  demo_points = [
    Pose(0.0, 0.0, WEST+deg2rad(36)),
    Pose(-s1, c1-1, WEST+deg2rad(36+72)),
    Pose(-s2, -c2-1, WEST+deg2rad(36+72*2)),
    Pose(s2, -c2-1, WEST+deg2rad(36+72*3)),
    Pose(s1, c1-1, WEST+deg2rad(36+72*4)),
    Pose(0, 0, NORTH),
  ]

  # Rotate in place.
  rot_points = [
    Pose(0.0, 0.0, EAST, 1.0, deg2rad(1)),
  ]

  shit_points = [
    Pose(3.0, 1,0, NORTH),
    Pose(1.0, 1.0, WEST)
  ]
  waypoints = shit_points

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

