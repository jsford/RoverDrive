#!/usr/bin/env python

import rospy
import numpy as np
import os.path as osp
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

if __name__=="__main__":
  rospy.init_node('goto')

  rate = rospy.Rate(30)

  mission_file_path = rospy.get_param('~mission')
  mission_file_path = osp.expanduser(mission_file_path)

  image_path = None
  image_exposures = None
  image_pans = None
  image_tilts = None
  

  with open(mission_file_path, 'r') as mission_file:
    print("Found mission file {}".format(mission_file_path))

    # Begin interpreting commands here!
    for command in mission_file.readlines():
      command = command.strip().split(' ')
      command = [c.strip(',"') for c in command if c != '']

      ready_to_go = (image_path != None and image_exposures != None and \
        image_pans != None and image_tilts != None)

      # Skip blank lines.
      if len(command) == 0:
        continue

      # Skip comment lines.
      if command[0] == '#':
        continue

      # Set image path.
      if command[0] == 'IMG_PATH':
        image_path = command[1]
        image_path = osp.expanduser(image_path)
        print("IMAGE PATH: {}".format(image_path))
        continue

      # Set image exposures.
      if command[0] == 'EXP':
        image_exposures = [int(x) for x in command[1:]]
        print("IMAGE EXPOSURES: {}".format(image_exposures))
        continue

      # Set pitcam pans.
      if command[0] == 'PAN':
        image_pans = [int(x) for x in command[1:]]
        print("IMAGE PANS: {}".format(image_pans))
        continue

      # Set pitcam tilts.
      if command[0] == 'TILT':
        image_tilts = [int(x) for x in command[1:]]
        print("IMAGE TILTS: {}".format(image_tilts))
        continue

      # If you have made it here, you better be ready to go!
      if not ready_to_go:
        print("ERROR: Make sure you have set IMG_PATH, EXP, PAN, and TILT before you try to ABS, REL, or CAP!")
        break

      if command[0] == 'ABS':
        goal_x = float(command[1])
        goal_y = float(command[2])
        goal_h = np.pi/180.0 * float(command[3])
        brink_en = command[4] == 'BRINK_ON'
        print("ABSOLUTE GOAL ({}, {}) @ {} BRINK {}".format(goal_x, goal_y, goal_h*180.0/np.pi, brink_en))
        continue

      if command[0] == 'REL':
        goal_x = float(command[1])
        goal_y = float(command[2])
        goal_h = np.pi/180.0 * float(command[3])
        brink_en = command[4] == 'BRINK_ON'
        print("RELATIVE GOAL ({}, {}) @ {} BRINK {}".format(goal_x, goal_y, goal_h*180.0/np.pi, brink_en))
        continue

      if command[0] == 'CAP':
        print("CAPTURE IMAGES!")
        continue

  while not rospy.is_shutdown():
    rate.sleep()

