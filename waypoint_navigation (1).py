#!/usr/bin/env python3

'''

This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
					
								

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

from distutils.log import error
from re import S
from tkinter import SEL_FIRST
from edrone_client.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time


class Edrone():
	"""docstring for Edrone"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]	

		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoint = [0,0,0]


		#Declaring a cmd of message type edrone_msgs and initializing values
		self.cmd = edrone_msgs()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500

		self.throttle_error = -20
		self.pitch_error = -2
		self.roll_error = -2

		self.max_throttle = 2000
		self.min_throttle = 1000

		self.max_pitch = 2000
		self.min_pitch = 1000

		self.max_roll = 2000
		self.min_roll = 1000

		self.positions = [[0,0,23],[2,0,23],[2,2,23],[-2,2,23],[-2,-2,23],[2,-2,23],[2,0,23],[0,0,23]]


		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
		self.Kp = [14,14,11]
		self.Ki = [0.5 ,0.5 ,0.5]
		self.Kd = [111,111,111]


		#-----------------------Add other required variables for pid here ----------------------------------------------

		self.prev_errors = [-2,-2,-20]     
		self.min_values = [1000, 1000, 1000]
		self.max_values = [2000, 2000, 2000]
		self.sum_of_errors = [0, 0, 0]
		self.change_in_error = [0, 0, 0]
		
		self.output = [0, 0, 0]

		self.sample_time = 0.060 # in seconds



		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', edrone_msgs, queue_size=100)
		
		#------------------------Add other ROS Publishers here-----------------------------------------------------

		#Get remaining topics from the pid_tune package
		self.alt_error_pub = rospy.Publisher('/alt_error', Float64, queue_size=1)
		self.pitch_error_pub = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.roll_error_pub = rospy.Publisher('roll_error', Float64, queue_size=1)





		#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------
		rospy.Subscriber('pid_tuning_pitch', PidTune, self.pitch_set_pid)
		rospy.Subscriber('pid_tuning_roll', PidTune, self.roll_set_pid)





		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE
	
	def change_pos(self,a,b,c):
		self.setpoint = [a,b,c]
	

	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)

	def check_point(current, target):
		x_reached = False
		y_reached = False
		z_reached = False
		if abs(current[0] - target[0]) < 0.2:
			x_reached = True
		if abs(current[1] - target[1]) < 0.2:
			y_reached = True
		if abs(current[2] - target[2]) < 0.2:
			z_reached = True
		
		if x_reached and y_reached and z_reached: return True 
		else: return False
			

	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX4 = 1500

		self.roll_value = 0
		self.pitch_value = 0
		self.throttle_value = 0

	
		
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		print('armed the drone')
		rospy.sleep(1)



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x

		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z




		
		#---------------------------------------------------------------------------------------------------------------



	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.06 # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.Ki * 0.008
		self.Kd[2] = alt.Kd * 0.3

	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------
	def pitch_set_pid(self, pitch):
		self.Kp[1] = pitch.Kp*0.06
		self.Ki[1] = pitch.Ki * 0.008
		self.Kd[1] = pitch.Kd * 0.3

	def roll_set_pid(self, roll):
		self.Kp[0] = roll.Kp*0.06
		self.Ki[0] = roll.Ki * 0.008
		self.Kd[0] = roll.Kd * 0.3

















	#----------------------------------------------------------------------------------------------------------------------


	def pid(self):
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	#	6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#																														self.cmd.rcPitch = self.max_values[1]
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	#	8. Add error_sum

		#computing errors
		self.throttle_error = -(self.setpoint[2] - self.drone_position[2])
		self.pitch_error = (-self.setpoint[1] + self.drone_position[1])
		self.roll_error = -(-self.setpoint[0] + self.drone_position[0])

		#computing error parameters
		self.change_in_error = [self.roll_error - self.prev_errors[0], self.pitch_error - self.prev_errors[1], self.throttle_error - self.prev_errors[2]]
		self.sum_of_errors += [self.roll_error, self.pitch_error, self.throttle_error]
		self.prev_errors = [self.roll_error, self.pitch_error, self.throttle_error]


		self.output[0] = int(self.Kp[0]*self.roll_error + self.Kd[0]*self.change_in_error[0] + self.Ki[0]*self.sum_of_errors[0])
		self.output[1] = int(self.Kp[1]*self.pitch_error + self.Kd[1]*self.change_in_error[1] + self.Ki[1]*self.sum_of_errors[1])
		self.output[2] = int(self.Kp[2]*self.throttle_error + self.Kd[2]*self.change_in_error[2] + self.Ki[2]*self.sum_of_errors[2])
	#------------------------------------------------------------------------------------------------------------------------

		print('output ', self.output[2])
		
		self.roll_value = 1500 + self.output[0]
		self.pitch_value = 1500 + self.output[1]
		self.throttle_value = 1500 + self.output[2]

		print('throttle value ', self.throttle_value)

		if self.roll_value > self.max_roll:
			self.cmd.rcRoll = self.max_roll
		elif self.roll_value < self.min_roll:
			self.cmd.rcRoll = self.min_roll
		else:
			self.cmd.rcRoll = self.roll_value

		if self.pitch_value > self.max_pitch:
			self.cmd.rcPitch = self.max_pitch
		elif self.pitch_value < self.min_pitch:
			self.cmd.rcPitch = self.min_pitch
		else:
			self.cmd.rcPitch = self.pitch_value

		if self.throttle_value > self.max_throttle:
			self.cmd.rcThrottle = self.max_throttle
		elif self.throttle_value < self.min_throttle:
			self.cmd.rcThrottle = self.min_throttle
		else:
			print('test')
			self.cmd.rcThrottle = self.throttle_value

		# if self.drone_position[0] - self.setpoint[0] < 0 and self.drone_position[2] < 24:
		# 	self.cmd.rcRoll = self.roll_value
		# 	print('roll value: ', self.cmd.rcRoll)
		# else:
		# 	self.cmd.rcRoll = -1*self.roll_value

		
		print('throttle: ', self.cmd.rcThrottle)
		print('roll: ', self.cmd.rcRoll)
		print('pitch: ', self.cmd.rcPitch)
		print(self.drone_position)
		self.command_pub.publish(self.cmd)
		self.alt_error_pub.publish(self.throttle_error)
		self.pitch_error_pub.publish(self.pitch_error)
		self.roll_error_pub.publish(self.roll_error)
	




if __name__ == '__main__':

	e_drone = Edrone()
	r = rospy.Rate(16.67) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	
	while not rospy.is_shutdown():
		for i, pos in enumerate(e_drone.positions):
			e_drone.setpoint = pos

			while not e_drone.check_point(e_drone.drone_position, e_drone.setpoint):
				e_drone.pid()
				r.sleep()
				continue
			
			print('reached ', e_drone.setpoint)

			
		r.sleep()
		
