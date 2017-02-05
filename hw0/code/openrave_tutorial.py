#!/usr/bin/env python

PACKAGE_NAME = 'hw0'

# Standard Python Imports
import os
import copy
import time
import numpy as np
import scipy

# OpenRAVE
import openravepy
#openravepy.RaveInitialize(True, openravepy.DebugLevel.Debug)


curr_path = os.getcwd()
relative_ordata = '/models'
ordata_path_thispack = curr_path + relative_ordata

#this sets up the OPENRAVE_DATA environment variable to include the files we're using
openrave_data_path = os.getenv('OPENRAVE_DATA', '')
openrave_data_paths = openrave_data_path.split(':')
if ordata_path_thispack not in openrave_data_paths:
  if openrave_data_path == '':
      os.putenv('OPENRAVE_DATA', ordata_path_thispack)
  else:
      os.putenv('OPENRAVE_DATA', '%s:%s'%(ordata_path_thispack, openrave_data_path))


class RoboHandler:
  def __init__(self):
    self.env = openravepy.Environment()
    self.env.SetViewer('qtcoin')
    self.env.GetViewer().SetName('Tutorial Viewer')
    self.env.Load('models/%s.env.xml' %PACKAGE_NAME)
    # time.sleep(3) # wait for viewer to initialize. May be helpful to uncomment
    self.robot = self.env.GetRobots()[0]
    

  #remove all the time.sleep(0) statements! Those are just there so the code can run before you fill in the functions

  # move in a straight line, depending on which direction the robot is facing
  def move_straight(self, dist):
    #TODO Fill in, remove sleep
    with self.env:
     translate = self.robot.GetTransform()
     self.robot.SetTransform(np.dot(translate, [ [1, 0, 0,dist], [0, 1, 0, 0], [0, 0, 1.0, 0.0], [0, 0, 0, 1.0] ]))
     time.sleep(0)

  # rotate the robot about the z-axis by the specified angle (in radians)
  def rotate_by(self, ang):
    #TODO Fill in, remove sleep
    rotate = self.robot.GetTransform()
    with self.env:
     rotate = np.dot(rotate, [[np.cos(ang * np.pi/180.), np.sin(ang * np.pi/180.),0, 0], [-np.sin(ang * np.pi/180.), np.cos(ang * np.pi/180.), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
     self.robot.SetTransform(rotate)
     time.sleep(0)

    # saves an image from above, pointed straight down
  def save_viewer_image_topdown(self, imagename):
    TopDownTrans = np.array([ [0, -1.0, 0, 0], [-1.0, 0, 0, 0], [0, 0, -1.0, 5.0], [0, 0, 0, 1.0] ])
    #seems to work without this line...but its in the tutorial, so I'll keep it here in case
    self.env.GetViewer().SendCommand('SetFiguresInCamera 1') # also shows the figures in the image
    I = self.env.GetViewer().GetCameraImage(640,480,  TopDownTrans,[640,640,320,240])
    scipy.misc.imsave(imagename + '.jpg',I)


  # go to each of the square corners, point towards the center, and snap a photo!
  def go_around_square(self):
    #TODO Fill in
    # with self.env:
    #  self.robot.SetTransform(np.identity(4)); 
    self.move_straight(1)
    self.rotate_by(90)
    self.move_straight(1)
    self.rotate_by(135)
    time.sleep(10)

    self.rotate_by(-45)
    self.move_straight(2)
    self.rotate_by(135)
    time.sleep(10)

    self.rotate_by(-45)
    self.move_straight(2)
    self.rotate_by(135)
    time.sleep(10)
    
    self.rotate_by(-45)
    self.move_straight(2)
    self.rotate_by(135)
    time.sleep(10)
    # with self.env:
    #  self.robot.SetTransform(np.identity(4)); 
  # a function to help figure out which DOF indices correspond to which part of HERB
  def figure_out_DOFS(self):
    #TODO Fill in, remove sleep
    for i in range(0,self.robot.GetDOF()-1):
      print self.robot.GetJointFromDOFIndex(i)
      #time.sleep(0)
  
  # put herb in self collision
  def put_in_self_collision(self):
    #TODO Fill in, remove sleep
    #lindex = self.robot.GetJoint('l_shoulder_pan_joint').GetDOFIndex()
    self.robot.SetDOFValues([2],[12], 0)
 
    '''   self.robot.SetDOFValues([  0.7,   1.9700000e+00,   2.74000000e+00,
         -8.726646e-01,  -2.22044605e-1,   0.00000000e+00,
         1.00000000e+00,  -2.11022302e-16,  -1.11022302e-16,
        -1.66533454e-16,   0.00000000e+00,   5.23598776e-01,
         0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
        -2.22044605e-16,   0.00000000e+00,   0.00000000e+00,
        -1.11022302e-16,  -1.11022302e-16,  -1.66533454e-16,
         0.00000000e+00,   0.00000000e+00,   0],0)
    '''
    time.sleep(0)
    with self.env:
     self.robot.SetTransform(np.identity(4)); 

  
      

if __name__ == '__main__':
  robo = RoboHandler()

  # Uncomment the following to make the script initialize the RoboHandler
  #  and drop you into an IPython shell.
  t = np.array([ [0, -1.0, 0, 0], [-1.0, 0, 0, 0], [0, 0, -1.0, 5.0], [0, 0, 0, 1.0] ])  
  robo.env.GetViewer().SetCamera(t)

  import IPython
  IPython.embed()

  # spin forever
  while True:
    time.sleep(1)
  
  
