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
		self.PID_range = PID(1.5,-.4, 0.0, 0.0)
		self.PID_theta = PID(0.0, 0.75, 0.0, 0.0)
		self.PID_rangeInter = PID(0.0, -.9, 0.0, 0.0)
		self.bearing = -1.0
		self.distance = -1.0
		self.inter_x = 0.0
		self.inter_y = 0.0
		self.x = 0.0
		self.y = 0.0
		self.theta = 0.0

	def handle_pose(self, msg):
		self.x = msg.pose.pose.position.x
		self.y = msg.pose.pose.position.y
		q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
		msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
		(_, _, self.theta) = euler_from_quaternion(q)	

	def get_vector(self, from_x, from_y, to_x, to_y):
		bearing = math.atan2(to_y - from_y, to_x - from_x)
		distance = math.sqrt((from_x - to_x)**2 + (from_y - to_y)**2)
		return (bearing, distance)
	
	def calibrate_angle(self, angle):
		out_angle = angle
		while out_angle < -math.pi:
			out_angle += math.pi * 2
		while out_angle > math.pi:
			out_angle -= math.pi * 2
		return out_angle

	
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
					
					#inter_bearing = theta + pi/4
					#inter_distance = (3 * sqrt(2)) / 2
					#inter_x = self.x + 1.5
					#inter_y = self.y + 1.5

					self.inter_x = self.x + math.cos(self.theta + math.pi/4.0) * ((3.0 * math.sqrt(2.0)) / 2.0)
					self.inter_y = self.y + math.sin(self.theta + math.pi/4.0) * ((3.0 * math.sqrt(2.0)) / 2.0)
					self.goal_x = self.x + 3.0 * math.cos(self.theta)
					self.goal_y = self.y + 3.0 * math.sin(self.theta)
						
					self.state = "inter"

				twist.angular.z = angular_speed
				twist.linear.x = linear_speed
				print "forward"
						
				
				print 'twist'
				print self.bearing
				print self.distance

			elif self.state == "inter":
				(bearing, distance) = self.get_vector(self.x, self.y, self.inter_x, self.inter_y)

				angle = self.calibrate_angle(self.theta - bearing)				
				
				angular_speed = self.PID_theta.get_output(angle)
				linear_speed = self.PID_rangeInter.get_output(distance)

				if linear_speed > .2:
					linear_speed = .2 
				#if linear_speed < .2:
				#	linear_speed = .2
				if angular_speed > .2:
					angular_speed = .2 
				if angular_speed < .2:
					angular_speed = .2
				
				twist.linear.x = linear_speed
				twist.angular.z = angular_speed

				print angular_speed
				print linear_speed
				print "x1 ", self.x
				print "y1 ", self.y
				print "x2 ", self.inter_x
				print "y2 ", self.inter_y
				print "theta ", self.theta
				print "Dis ", distance
				print "Bear ", bearing
				print "PID Value ", self.theta - distance
				
				distanceError = abs(self.PID_range.previous_error)
				print "error ", distanceError

				if distance <= .5:

					twist.linear.x = 0.0
					twist.angular.z = 0.0
					self.state = "kickPos"

					print "Stop"
					
					

			elif self.state == "kickPos":
				(bearing, distance) = self.get_vector(self.x, self.y, self.goal_x, self.goal_y)
				angular_speed = self.PID_theta.get_output(self.theta - bearing)
				linear_speed = self.PID_rangeInter.get_output(distance)

				if linear_speed > .2:
					linear_speed = .2 
				#if linear_speed < -.2:
				#	linear_speed = -.2
				#if angular_speed > .2:
				#	angular_speed = .2
				
				twist.linear.x = linear_speed
				twist.angular.z = angular_speed

				print angular_speed
				print linear_speed
				print "x1dac ", self.x
				print "y1adcd ", self.y
				print "x2 ", self.goal_x
				print "y2 ", self.goal_y
				print "theta ", self.theta
				print "Dis ", distance
				print "Bear ", bearing
				print "PID Value ", self.theta - distance
				
				distanceError = abs(self.PID_range.previous_error)
				print "error ", distanceError
				

				if distance <= .15:
					twist.linear.x = 0.0
					self.time = rospy.get_time()
					self.state = "turn"

			elif self.state == "turn":
					twist.angular.z = -1.0
					while (rospy.get_time() - self.time) <= 7:
						twist.angular.z = 0.0
						self.state = "lineUp"
					

					print "Stop"
				
			elif self.state == "lineUp":
				

				print "LINEUPPPPPPPP"
					


				angular_speed = self.PID_bearing.get_output(self.bearing)
				#linear_speed = self.PID_range.get_output(self.distance)
				
				if linear_speed > .2:
					linear_speed = .2 
				if linear_speed < -.2:
					linear_speed = -.2
					
				#if self.bearing < 0:
				#	self.state = "search"
				#	twist.linear.x = 0

				if 305 <= self.bearing <= 340:
					twist.linear.x = 0
					self.time = rospy.get_time()
					self.state = "kick"
					
				twist.angular.z = angular_speed
				#twist.linear.x = linear_speed
				
				print angular_speed
				print self.bearing
				print self.distance
				
			
			elif self.state == "kick":
				print "ASDASDSUIDHUIASDASDASDASASASAASDASD"
				twist.linear.x = 1
				if rospy.get_time() - self.time >= 2.5:
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
