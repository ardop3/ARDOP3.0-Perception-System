from sensor_msgs.msg import	Image
#not sure whether PointCloud2 is required or not
from sensor_msgs.msg import	PointCloud2
import rospy,cv_bridge,cv2



class Kinect_depth(object):
	"""docstring for Kinect_depth
		get_depth returns depth value"""
	def __init__(self, x,y):
		
		self.x = x
		self.y = y
		#first it is initialised to 0,it will get updated later
		self.depth=0

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
		print(self.depth)



	def get_depth(self):
		#print("3")
		return self.depth

#in your diplay detection function
#initialising kinect_depth object and passing centroid value
if __name__ == '__main__':
	while not rospy.is_shutdown():

		with open('place_list.dat','r') as f:
			output_list1 = f.read().strip().split(',')
			place_depth =  Kinect_depth(int(output_list1[1]),int(output_list1[2]))
			place_depthval=place_depth.get_depth()
			print(place_depthval)
			

		with open('pick_list.dat','r') as f:
			output_list2 = f.read().strip().split(',')
			pick_depth =  Kinect_depth(int(output_list2[1]),int(output_list2[2]))
			pick_depthval=place_depth.get_depth()
			print(pick_depthval)
						
		
    	#returns depth value.
		#depthval=depth.get_depth()
		#print(depthval)
		#add print statement if you want to print the value
		rospy.spin()




