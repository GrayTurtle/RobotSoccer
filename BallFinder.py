#!/usr/bin/env python
import roslib
roslib.load_manifest('beginner_tutorials')
import rospy
import math
import cv2
from sensor_msgs.msg import Image, LaserScan
from beginner_tutorials.msg import BallLocation
from cv_bridge import CvBridge, CvBridgeError

class Detector:

	def __init__(self):
		# The image publisher is for debugging and figuring out
		# good color values to use for ball detection
		self.impub = rospy.Publisher('/ball_detector/image', Image, queue_size=1)
		self.locpub = rospy.Publisher('/ball_detector/ball_location', BallLocation,
					     queue_size=1)
		self.bridge = CvBridge()
		self.bearing = -1
		self.distance = -1

		rospy.Subscriber('/camera/rgb/image_raw', Image, self.handle_image, queue_size=1 , buff_size=2**24)
		rospy.Subscriber('/scan', LaserScan, self.handle_scan)
	
	def handle_image(self, msg):
		try:
			image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		except CvBridgeError, e:
			print e
		# Find the average column of the bright yellow pixels
		# and store as self.bearing. Store -1 if there are no
		# bright yellow pixels in the image.
		# Feel free to change the values in the image variable
		# in order to see what is going on
		(row, col, chan) = image.shape
		
		lB = 20
		hB = 70
		lG = 120
		hG = 250
		lR = 120
		hR = 250
		colSum = 0
		numOfYel = 0

		for i in range(0, row, 3):
			for j in range(0,col, 3):
				if lB < image[i, j, 0] < hB and lG < image[i, j, 1] < hG and lR < image[i,j,2] < hR:
					image[i,j,0] = 0
					image[i,j,1] = 0
					image[i,j,2] = 255
					
					colSum = j + colSum
					numOfYel = numOfYel + 1

		if numOfYel >= 75:  #***might have to adjust for soccer tournament
			self.bearing = colSum/numOfYel
		elif numOfYel < 75:
			self.bearing = -1


		print numOfYel
		# Here we publish the modified image; it can be
		# examined by running image_view
		self.impub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))

	def handle_scan(self, msg):
		# If the bearing is valid, store the corresponding range
		# in self.distance.  Decide what to do if range is NaN.
		
		if self.bearing > 0:
			self.distance = msg.ranges[self.bearing]
			if math.isnan(self.distance):
				print 'Ball not in range'
			else: 
				print 'Bearing: ', self.bearing
				print 'Range: ', self.distance
		if self.bearing < 0:
			self.distance = -1
			print 'No ball found'	

	def start(self):
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			location = BallLocation()
			location.bearing = self.bearing
			location.distance = self.distance
			self.locpub.publish(location)
			rate.sleep()

rospy.init_node('ball_detector')
detector = Detector()
detector.start()
