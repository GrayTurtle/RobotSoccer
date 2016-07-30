#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from beginner_tutorials.msg import BallLocation
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import math
class Puppy:
	
	def __init__(self):
		self.pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)
	
		rospy.Subscriber("/ball_detector/ball_location",BallLocation, self.ballLocate)

		rospy.Subscriber('/odom', Odometry, self.handle_pose)

		self.state = "search"
		self.PID_bearing = PID(320.0,.002,0.0,0.0)
		self.PID_range = PID(1.5,-.4,0.0,0.0)
		self.bearing = -1
		self.distance = -1

	def handle_pose(self, msg):
		self.x = msg.pose.pose.position.x
		self.y = msg.pose.pose.position.y
		q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
		msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
		(_, _, self.theta) = euler_from_quaternion(q)	

	
	def ballLocate(self,msg):
		self.bearing = msg.bearing
		self.distance = msg.distance
		#print self.bearing
	
	def run(self):
		rate = rospy.Rate(10)
		twist = Twist()
		while not rospy.is_shutdown():
			
			if math.isnan(self.distance):
				self.distance = -1
			if self.state == "search":
				twist.angular.z = 0.25
				twist.linear.x = 0
				
				if self.bearing > 0 and self.distance > 0:
					print "Approach"
					print self.bearing
					self.state = "approach"	
				else:
					print "searching..."

			elif self.state == "approach":

				if self.bearing < 0:
					self.state = "search"
					twist.linear.x = 0

				angular_speed = self.PID_bearing.get_output(self.bearing)
				linear_speed = self.PID_range.get_output(self.distance)
				
				if linear_speed > .2:
					linear_speed = .2 
				if linear_speed < -.2:
					linear_speed = -.2

				if 1.4 <= self.distance <= 1.8 and 310 <= self.bearing <= 325:
					twist.linear.x = 0
					self.time = rospy.get_time()
					self.state = "kick"

				twist.angular.z = angular_speed
				twist.linear.x = linear_speed
				print "forward"
						
				
				print 'twist'
				print self.bearing
				print self.distance
			
			elif self.state == "kick":
				twist.linear.x = 1
				if rospy.get_time() - self.time >= 2.2:
					twist.linear.x = 0
					self.state = "search"
				
			self.pub.publish(twist)				
									
			rate.sleep()


class PID:
	
	def __init__(self, goal, kp, ki, kd):
		self.goal = goal
		self.kp = kp
		self.ki = ki
		self.kd = kd
		self.previous_error = 0.0
		self.integral = 0.0
	
	def get_output(self, measurement):
		error = self.goal - measurement
		self.integral = self.integral + error
		derivative = error - self.previous_error
		output = self.kp*error + self.ki*self.integral + self.kd*derivative
		self.previous_error = error
		return output


		

rospy.init_node('robot_puppy')
puppy = Puppy()
puppy.run()
