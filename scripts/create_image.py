#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int16

from collections import deque

import numpy as np
import argparse

import imutils
import time
import os


HEADER = '\033[95m'
OKBLUE = '\033[94m'
OKGREEN = '\033[92m'
WARNING = '\033[93m'
FAIL = '\033[91m'
ENDC = '\033[0m'
BOLD = '\033[1m'
UNDERLINE = '\033[4m'



MAIN_PATH = os.path.abspath(os.path.dirname(__file__))
class image_converter:

  def __init__(self):

    
    
    self.h = 0 
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_color",Image,self.callback)



  def callback(self,data):


    	global cv_image
 

    	cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    	for i in range(0, 1):

  			cv_image_path = '%s/rgb/%d.jpg'%(MAIN_PATH, i)



        cv2.imwrite(cv_image_path, cv_image)
        self.h = self.h+1

        rospy.signal_shutdown("image_created")
        exit()
        cv2.destroyAllWindows()  
        sys.exit()    



def main(args):
  ic = image_converter()

  rospy.init_node('image_converter', anonymous=True)
  
  #rospy.init_node('index_sender', anonymous = True)


  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()




if __name__ == '__main__':

  print( BOLD +UNDERLINE + WARNING +   "Initilizing the Computer Vision System" +ENDC )
  print( OKGREEN +" ----------------------------------- " + ENDC)
  print( OKBLUE + "Capturing the Image" + ENDC )
  print( OKGREEN +" ----------------------------------- " + ENDC)
  print(OKGREEN+ "wait for 5 seconds unitl the Image is Created" + ENDC )

  print( OKGREEN +" ----------------------------------- " + ENDC)

  time.sleep(5)

  print( BOLD +UNDERLINE + WARNING +   "Initilizing YOLO V3 for Object Detection" +ENDC )

  print( OKGREEN +" ----------------------------------- " + ENDC)


  main(sys.argv)


'''
      if (self.h == 0):

        cv2.imwrite(cv_image_path, cv_image)
        self.h = self.h+1
        cv2.destroyAllWindows()

'''