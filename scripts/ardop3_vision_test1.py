'''
./darknet detector demo cfg/coco.data cfg/yolov3.cfg weights/yolov3.weights -c 0
./darknet detect cfg/yolov3.cfg weights/yolov3.weights data/0.jpg
'''

import os
import sys
import math
import time
from subprocess import Popen, PIPE
import cv2

#import speech


import roslib
#roslib.load_manifest('my_package')

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



# Get this path
MAIN_PATH = os.path.abspath(os.path.dirname(__file__))

# Append sys paths to find the modules
sys.path.append(os.path.abspath(os.path.dirname(__file__))+'/kinect/wrappers/python/')
sys.path.append(os.path.abspath(os.path.dirname(__file__))+'/darknet')

# Import darknet
from darknet import darknet
DARKNET_PATH = darknet.get_path()

# Import kinect
import demo_cv2_sync as kinect
KINECT_PATH = kinect.get_path()

# Kinect dimensions
KINECT_WIDTH = 640
KINECT_HEIGHT = 480
CAMERA_Z_OFFSET = 0.84
CAMERA_X_OFFSET = 0.06
CAMERA_TILT = 73

# Number of Kinect Samples
SAMPLES = 1

# List of objects
LABELS = None

# Formatting
HEADER = '\033[95m'
OKBLUE = '\033[94m'
OKGREEN = '\033[92m'
WARNING = '\033[93m'
FAIL = '\033[91m'
ENDC = '\033[0m'
BOLD = '\033[1m'
UNDERLINE = '\033[4m'

################ COMMON ################
def cd_main():
	os.chdir(MAIN_PATH)

def cd_darknet():
	os.chdir(DARKNET_PATH)

def cd_kinect():
	os.chdir(KINECT_PATH)

################ KINECT ################
def calibrate_rgb_image(rgb):
	cropped = rgb[30:480, 8:600]
	resized_rgb_image = cv2.resize(cropped, (640, 480))
	return resized_rgb_image

def display_rgbd_map(rgb, depth, calibrate=True):
	alpha = 0.5

	depth = cv2.cvtColor(depth, cv2.COLOR_GRAY2BGR)
	depth = cv2.cvtColor(depth, cv2.COLOR_HSV2BGR)
	if calibrate: rgb = calibrate_rgb_image(rgb)

	merged = cv2.addWeighted(depth, alpha, rgb, 1-alpha, 0) 
	cv2.imshow('RGBD Map', merged)

	return merged



def display_live():
	cd_kinect()
	while (1):
		rgb = kinect.get_rgb()
		rgb_cal = calibrate_rgb_image(kinect.get_rgb())
		cv2.imshow('RGB', rgb_cal)

		depth = kinect.get_depth()
		depth_map = cv2.cvtColor(depth, cv2.COLOR_GRAY2BGR)
		depth_map = cv2.cvtColor(depth_map, cv2.COLOR_HSV2BGR)
		cv2.imshow('Depth', depth_map)

		display_rgbd_map(rgb, depth)

		# Exit on pressing spacebar 
		if cv2.waitKey(1) & 0xFF == ord(' '): 
			break

	cv2.destroyAllWindows()



def load_labels():
	global LABELS
	cd_main()
	with open('labels.txt') as f:
		LABELS = [i.split('\n')[0] for i in f.readlines()]
	print LABELS

def load_graph():
	print WARNING + BOLD + '\nLoading YOLO network...' + ENDC
	cd_darknet()
	darknet.load_yolo(3)
	print BOLD + OKGREEN + 'Done' + ENDC

def get_images_list(folder):
	images_list = []
	for root, dirs, files in os.walk(folder):
		for file in files:
			images_list.append(os.path.abspath(os.path.join(root, file)))
	return images_list

def detect_objects():
	# Get list of RGB images to run detection
	cd_main()
	rgb_paths = get_images_list('./rgb')
	depth_paths = get_images_list('./depth')

	# Detect objects
	print WARNING + BOLD + '\nRunning YOLO network...' + ENDC
	results = []

	cd_darknet()
	for rgb_path, depth_path in zip(rgb_paths, depth_paths):
		print 'Running YOLO on: %s'%rgb_path
		yolo_result = [darknet.yolo_detect(rgb_path)] + [rgb_path] + [depth_path]
		results.append(yolo_result)
	
	print BOLD + OKGREEN + 'Done' + ENDC
	return results

def display_detections(path, bb, label,img=False, from_file=True):
	cd_main()
	if img is None: img = cv2.imread(path)
	if not os.path.exists('%s/detection'%MAIN_PATH): os.mkdir('%s/detection'%MAIN_PATH)
	detection_path = '%s/detection/detect.jpg'%(MAIN_PATH)
	sx = bb[0] - bb[2]/2
	sy = bb[1] - bb[3]/2
	ex = bb[0] + bb[2]/2
	ey = bb[1] + bb[3]/2

	cv2.rectangle(img, (sx, sy), (ex, ey), (0,255,0), 3)
	cv2.circle(img, (bb[0], bb[1]), 10, (0,0,255), -1)
	font = cv2.FONT_HERSHEY_SIMPLEX
	text = "%s"%(label.title())
	print text
	if(text == "Apple" or text == "Orange"):
		text = "Ball" 


	if ( label == "cup" or label == "bowl" or label == "spoon"):

		with open("place_list2.dat", "w+") as f:
			f.write(str(["place", bb[0], bb[1],"place"]))	


	if (label == "Ball" or label == "sports ball"  or label == "Apple" or label == "apple"):
		with open("pick_list2.dat", "w+") as f:
			f.write(str(["pick", bb[0], bb[1],"pick"]))




	# str(base_xyz[0]), str(base_xyz[1]), str(base_xyz[2]))
	cv2.putText(img, text, (bb[0],bb[1]), font, 1 , (0,255,0), 1, cv2.LINE_AA)
	cv2.imshow('Detections', img)
	cv2.imwrite(detection_path,img)

	return img


def get_centroid(bounding_box):

	cnt1 = bounding_box[0]
	cnt2 = bounding_box[1]

	return cnt1, cnt2 


def process_results(results):
	object_info = [None]
	object_id = 1
	global LABELS

	for result, rgb_path, depth_path in results:
		img = None
		for label, confidence, box in result:
			if not label in LABELS:
				continue
			print box[0]
			text = "%s"%(label.title())
			print text
			if(text == "Apple"):
				text = "Ball"
			if(text == "Orange"):
			 	text = "Ball"



			centroid_locations = get_centroid(box)

			img = display_detections(rgb_path, box, label, img=img)
			print '\nObject %d: %s%s%s%s'%(object_id, FAIL, BOLD, text, ENDC)

			confidence_int = int(confidence*100)
			print 'Confidence: %d%%'%confidence_int
			
			#print 'Coordinates: ', base_xyz

			object_info.append((label, confidence_int, centroid_locations))
			object_id += 1

		with open("detection.dat", "w+") as f:
			f.write(str( object_info ))		

	print '\n>>> Enter spacebar to continue <<<'

	cv2.waitKey(0)
	cv2.destroyAllWindows()




def setup():
	# Get RGBD samples from Kinect
	#generate_kinect_images()

	

	load_labels()
	load_graph()

def run():
	# Run YOLO and get predictions
	results = detect_objects()

	# Process results
	return process_results(results)

if __name__ == '__main__':

	LIVE_DEMO = True
	
	if LIVE_DEMO:
		# speech.speak("Hello World! I am Autonomous Robot Development Open Source Platform. But you can call me Ardaup.")
		# speech.speak("I have been developed by Aditya, Akshay and Heethesh under the guidance of Harish V. Mekali!")
		# speech.speak("I can tell what things you are pointing at me and pick them up using my arm!")
		# speech.speak("Please point an object at me and I will tell you what it is.")
		
		setup()
		# speech.speak("Please wait while I recognize objects using the YOLO version three framework.")
		# speech.speak("I can see where objects are using the Kinect three sixty sensor.")

		print run()

		# speech.speak("Now I shall proceed to try to pick up the object")

	else:
		display_live()
