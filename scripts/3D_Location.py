from sensor_msgs.msg import	Image
#not sure whether PointCloud2 is required or not
from sensor_msgs.msg import	PointCloud2
import rospy,cv_bridge,cv2
import math
import numpy as np
import sys

class Kinect_depth(object):
	"""docstring for Kinect_depth
		get_depth returns depth value"""
	def __init__(self, x,y, index):
		
		self.x = x
		self.y = y
		#first it is initialised to 0,it will get updated later
		self.depth=0
		self.index = index
		self.x_constant = -7
		self.y_constant = 17
		self.t =0
		
		#node intitalisation.only one node per script!
		rospy.init_node('ros_bridge')
		
		#ros bridge to convert messages to opencv format
	 	self.ros_bridge = cv_bridge.CvBridge()

	 	#after launching open ni by roslaunch openni_launch openni.launch 
	 	# run this command rosrun rqt_reconfigure rqt_reconfigure and then go to driver and mark yes for depth_registration,
	 	#otherwise you wont get depth data for subscriber below
	 	#print("1")
	 	self.image_sub1= rospy.Subscriber('/camera/depth_registered/image_raw', Image,self.image_callback)

	 	#callback to find depth
	def image_callback(self,msg):
		image1= self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
		#to find depth 
		
		self.depth=image1[self.y,self.x]

		if (self.t < 5): 

			if (self.index == 1):
				print("pick")
				print(self.depth)
				self.determine_pick_coordinate()
				print("-----------------")

			if (self.index == 2):
				print("place")
				print(self.depth)
				self.determine_place_coordinate()
				print("------------")

			self.t=self.t+1
		rospy.signal_shutdown("cordinates have been determined ")
		sys.exit()





		
	def get_depth(self):
		#print("3")
		return self.depth


	def determine_pick_coordinate(self):

		'''

		X = self.x - 320
		Y = self.y - 200
		pick_coordinate = []
		alpha  = 43/480*Y * math.pi/180
		beta = 57/640*X * math.pi/180
		theta_s1  = (math.pi)/2
		theta_s0 = 0
		x_co = self.depth*math.sin(math.pi + alpha)*math.cos(theta_s0 +beta)
		y_co= self.depth*math.sin(math.pi+alpha)*math.sin(theta_s0+beta)
		z_co = self.depth*math.cos(math.pi +alpha)
		pick_coordinate = [x_co,y_co,z_co]
		print(pick_coordinate)
		'''

		x_co = 0.18*self.x - 23 + self.x_constant
		y_co = -0.156*self.y + 73.5 +self.y_constant
		z_co = 38
		pick_coordinate = [x_co,y_co,z_co]
		with open("pick_coordinates.dat", "w+") as f: 
			f.write(str(("pick_coordinate",x_co, y_co, z_co,"pick_coordinate")))
		print(pick_coordinate)

		


	def determine_place_coordinate(self):

		x_co = 0.18*self.x - 23 + self.x_constant
		y_co = -0.156*self.y + 73.5 +self.y_constant
		z_co = 33
		place_coordinate = [x_co,y_co,z_co]
		with open("place_coordinates.dat", "w+") as f: 
			f.write(str(("place_coordinates",x_co, y_co, z_co,"place_coordinates")))
		print(place_coordinate)







#in your diplay detection function
#initialising kinect_depth object and passing centroid value
if __name__ == '__main__':

	while not rospy.is_shutdown():

		'''

		with open('/home/ardopgen2/ardop3_ws/src/ardop3_moveit_config/nodes/ardop3_vision/place_list2.dat','r') as f:
			output_list1 = f.read().strip().split(',')
			place_depth =  Kinect_depth(int(output_list1[1]),int(output_list1[2]),2)
			place_depthval=place_depth.get_depth()
			print(place_depthval)
		
		'''

		with open('/home/ardopgen2/ardop3_ws/src/ardop3_moveit_config/nodes/ardop3_vision/pick_list2.dat','r') as f:
			output_list2 = f.read().strip().split(',')
			pick_depth =  Kinect_depth(int(output_list2[1]),int(output_list2[2]),1)
			pick_depthval=pick_depth.get_depth()
			print(pick_depthval)
		
		'''				
		
    	#returns depth value.
		#depthval=depth.get_depth()
		#print(depthval)
		#add print statement if you want to print the value
		'''
		rospy.spin()