#!/usr/bin/env python

import rospy
import time
import numpy as np
import os
import os.path as osp
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pitranger.srv import *
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from PIL import Image
import piexif

class RobotExecutive:
  def __init__(self):
    self.image_path = None
    self.image_exposures = None
    self.image_pans = None
    self.image_tilts = None
    self.default_pan = None
    self.default_tilt = None

    # Count image capture locations
    self.capture_idx = 0

    # Capture odom messages and use them to set the robot state.
    self.state = None
    self.odom_sub = rospy.Subscriber('/whereami/odom/', Odometry, self.odom_callback)
    self.wait_for_odom()

    # Watch the brinkmanship range estimate to safeguard while driving.
    self.brink_range = None
    self.brink_sub = rospy.Subscriber('/brink/out/range', Float64, self.brink_callback)
    self.wait_for_brink()

  def is_initialized(self):
      return (self.image_path != None and self.image_exposures != None and \
              self.image_pans != None and self.image_tilts != None and \
              self.default_pan != None and self.default_tilt != None)

  def set_image_path(self, path):
    if not os.path.exists(path):
      os.mkdir(path)
    self.image_path = path

  def set_image_exposures(self, exposures):
    self.image_exposures = exposures

  def set_image_pans(self, pans):
    self.image_pans = pans

  def set_image_tilts(self, tilts):
    self.image_tilts = tilts

  def set_default_pan(self, p):
    print("SET DEFAULT PAN")
    self.default_pan = p
    if self.default_tilt is not None:
      self._set_pan_tilt(self.default_pan, self.default_tilt)

  def set_default_tilt(self, t):
    print("SET DEFAULT TILT")
    self.default_tilt = t
    if self.default_pan is not None:
      self._set_pan_tilt(self.default_pan, self.default_tilt)

  def odom_callback(self, msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    ori = msg.pose.pose.orientation
    q = np.array([ori.x, ori.y, ori.z, ori.w])
    _,_,yaw = euler_from_quaternion(q)
    self.state = np.array([x, y, yaw])

  def wait_for_odom(self):
    if self.state is None:
      time.sleep(0.1)

  def brink_callback(self, msg):
    self.brink_range = msg.data

  def wait_for_brink(self):
    if self.brink_range is None:
      time.sleep(0.1)

  def goto_absolute(self, goal_x, goal_y, goal_h, brink_en):
    time.sleep(1.0)
    pass

  def goto_relative(self, goal_x, goal_y, goal_h, brink_en):
    new_x = self.state[0] + goal_x
    new_y = self.state[1] + goal_y
    new_h = self.state[2] + goal_h
    return self.goto_absolute(new_x, new_y, new_h, brink_en)

  def _set_pan_tilt(self, p, t):
    rospy.wait_for_service('/pitcam/set_pan_tilt')
    try:
      set_pt = rospy.ServiceProxy('/pitcam/set_pan_tilt', SetPanTilt)
      set_pt(p, t)
    except rospy.ServiceException as e:
      print("ERROR: SetPanTilt service call failed: %s"%e)

  def _capture_image(self, e):
    print("CAPTURING IMAGE")
    rospy.wait_for_service('/pitcam/capture')
    try:
      capture = rospy.ServiceProxy('/pitcam/capture', PitCamCapture)
      resp = capture(e)
      return resp
    except rospy.ServiceException as e:
      print("ERROR: PitCamCapture service call failed: %s"%e)
    return None

  def capture(self):
    for t in self.image_tilts:
      for p in self.image_pans:

        # Aim the camera and wait for it to stabilize.
        self._set_pan_tilt(p, t)
        time.sleep(1.0)

        # Loop over exposure bracket
        for e in self.image_exposures:

          # Quit if ros is dead.
          if rospy.is_shutdown():
            return 

          # Capture an image!
          img_data = self._capture_image(e)
          print("PAN {} TILT {} EXP {}".format(img_data.pan_deg, img_data.tilt_deg, img_data.exposure_us))
          if img_data.image.encoding != "RGB8":
            print("GOTO ERROR: Pitcam image encoding {} not recognized!".format(img_data.image.encoding))
          else:
            img = Image.frombytes('RGB', (img_data.image.width,img_data.image.height), img_data.image.data, 'raw')
            img_name = "{:03}_{:+03}_{:+03}_{:09}.jpg".format(self.capture_idx, p, t, e)
            img_path = osp.join(self.image_path, img_name)
            print(img_path)
            
            # Write metadata into exif
            exif_dict = {"0th":zeroth_ifd, "Exif":exif_ifd, "GPS":gps_ifd, "1st":first_ifd, "thumbnail":thumbnail}
            exif_bytes = piexif.dump(exif_dict)
            exif_bytes = []

            # Save image as jpg with exif
            img.save(img_path, exif=exif_bytes)

    # Return the camera to its default position.
    self._set_pan_tilt(self.default_pan, self.default_tilt)
    # Count captures for organizing images.
    self.capture_idx += 1

if __name__=="__main__":
  rospy.init_node('goto')

  mission_file_path = rospy.get_param('~mission')
  mission_file_path = osp.expanduser(mission_file_path)

  robot = RobotExecutive()

  with open(mission_file_path, 'r') as mission_file:
    print("Found mission file {}".format(mission_file_path))

    # Begin interpreting commands here!
    for line_number, command in enumerate(mission_file.readlines()):

      if rospy.is_shutdown():
        break

      command = command.strip().split(' ')
      command = [c.strip(',"') for c in command if c != '']

      # Skip blank lines.
      if len(command) == 0:
        continue

      # Skip comment lines.
      if command[0][0] == '#':
        continue

      # Set image path.
      if command[0] == 'IMG_PATH':
        image_path = command[1]
        image_path = osp.expanduser(image_path)
        robot.set_image_path(image_path)
        continue

      # Set image exposures.
      if command[0] == 'EXP':
        image_exposures = [int(x) for x in command[1:]]
        robot.set_image_exposures(image_exposures)
        continue

      # Set pitcam pans.
      if command[0] == 'PAN':
        image_pans = [int(x) for x in command[1:]]
        robot.set_image_pans(image_pans)
        continue

      # Set pitcam tilts.
      if command[0] == 'TILT':
        image_tilts = [int(x) for x in command[1:]]
        robot.set_image_tilts(image_tilts)
        continue

      if command[0] == 'DEFAULT_PAN':
        robot.set_default_pan(int(command[1]))
        continue

      if command[0] == 'DEFAULT_TILT':
        robot.set_default_tilt(int(command[1]))
        continue

      # If you have made it here, you better be ready to go!
      if not robot.is_initialized():
        print("GOTO ERROR [Line {}]: Make sure you have set IMG_PATH, EXP, PAN, TILT, DEFAULT_PAN, and DEFAULT_TILT before you try to ABS, REL, or CAP!".format(line_number))
        break

      # Drive to an absolute pose.
      if command[0] == 'ABS':
        goal_x = float(command[1])
        goal_y = float(command[2])
        goal_h = np.pi/180.0 * float(command[3])
        brink_en = command[4] != 'BRINK_OFF'
        robot.goto_absolute(goal_x, goal_y, goal_h, brink_en)
        continue

      # Drive to a relative pose.
      if command[0] == 'REL':
        goal_x = float(command[1])
        goal_y = float(command[2])
        goal_h = np.pi/180.0 * float(command[3])
        brink_en = command[4] != 'BRINK_OFF'
        robot.goto_relative(goal_x, goal_y, goal_h, brink_en)
        continue

      # Capture pitcam images.
      if command[0] == 'CAP':
        robot.capture()
        continue

      # Unrecognized command!
      print("ERROR: Unrecognized command \"{}\" on line {}. Skipping.".format(command[0], line_number))
