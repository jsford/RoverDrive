import numpy as np

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
