#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt32,Float64,Bool
import numpy as np
import time
import math
import sys
from select import select
import termios
import tty
import matplotlib.pyplot as plt
import atexit

turn_mode = rospy.get_param('/autonav/turn_mode')
speed_mode = rospy.get_param('/autonav/speed_mode')

if (speed_mode is "fast"):
	straight_speed = 1630
	turn_speed = 1540
	reverse_speed = 1440
	time_delay = 0.0
else:
	straight_speed = 1560
	turn_speed = 1560
	reverse_speed = 1440
	time_delay = 1.5

class Autonav:
	def __init__(self):
		rospy.init_node('autonav')

		self.depth_error = rospy.Subscriber('/depth_controller_error', Float64, self.depth_callback)
		self.convergence_error = rospy.Subscriber('/convergence_controller_error', Float64, self.convergence_callback)

		self.ball_detected = rospy.Subscriber('/ball_detected', Float64, self.ball_detection_callback)
		self.ball_position = rospy.Subscriber('/ball_location', Float64, self.ball_position_callback)

		self.ir_sensor = rospy.Subscriber('/ir_raw', UInt32, self.ir_sensor_callback)

		self.stop_sign = rospy.Subscriber('/stop_sign', Bool, self.stop_sign_callback)

		self.steering_input = rospy.Publisher('/servo_raw', UInt32, queue_size=10)
		self.velocity_input = rospy.Publisher('/velocity_raw', UInt32, queue_size=10)

		self.depth_turn = rospy.Subscriber('/depth_turn_indicator', Bool, self.depth_turn_callback)
		self.imu_turn = rospy.Publisher('/imu_turn', Bool, queue_size=10)

		self.crash = rospy.Subscriber('/crash', Bool, self.crash_callback)

		self.imu = rospy.Subscriber('/imu/data', Imu, self.imu_callback, queue_size=1)

		self.steering_pid = PIDController(0.1, 0.0, 0.0, 1500, 1000, 2000)

		self.controller_time = time.time()

		self.input_data = []
		self.output_data = []
		self.time_array = []

		self.turn = False
		self.stop_sign = False
		self.start = False

		self.depth_error = 0.5
		self.convergence_error = 0.5
		self.is_ball_detected = False
		self.ball_position = None
		self.imu_data = None

		self.imu_turn_value = False
		self.previous_yaw = None
		self.current_yaw = None
		self.yaw = None

		self.depth_turn_value = False

		self.desired_time = None

		self.crash = False

		self.stop = False

		self.velocity_input.publish(UInt32(1500))
		self.steering_input.publish(UInt32(1500))  

		self.settings = self.saveTerminalSettings()
		self.key_timeout = rospy.get_param("~key_timeout", 0.5)

		t_end = time.time() + 60 * 5.0

		while ((time.time() < t_end) and (not rospy.is_shutdown())):
			self.steering_input.publish(UInt32(1500))
			self.velocity_input.publish(UInt32(1500))
			rospy.loginfo('waiting to start...')
			key = self.getKey(self.settings, self.key_timeout)
			if (key == 'a'):
				rospy.loginfo('GO was pressed')
				break

		rospy.loginfo('done')
		self.start = True
		self.crash_ignore_time = time.time()

	def run(self):
		if (self.start):
			if (self.crash is True) and ((time.time() - self.crash_ignore_time) > 1):
				self.velocity_input.publish(UInt32(1500))
				self.steering_input.publish(UInt32(1500))
				time.sleep(2)
				self.velocity_input.publish(UInt32(reverse_speed))
				time.sleep(1)
				self.velocity_input.publish(UInt32(1500))
				time.sleep(2)
				self.depth_turn_value = False
				self.imu_turn_value = False
				self.crash = False

			elif ((self.depth_turn_value is True) or (self.imu_turn_value is True)) and (turn_mode is not "straight"):
				if self.desired_time is None:
					rospy.loginfo('TIME: {}'.format(self.desired_time))
					self.desired_time = time.time() + time_delay
				elif time.time() >= self.desired_time:
					rospy.loginfo('IMU turn initiated')
					rospy.loginfo('yaw: {}'.format(self.yaw))
					if self.previous_yaw is None:
						self.previous_yaw = self.yaw
						self.imu_turn_value = True
					else:
						self.current_yaw = self.yaw
						self.imu_turn_value = not self.is_turn_finsihed()

					if self.imu_turn_value is False:
						self.previous_yaw = None
						self.desired_time = None

					if (turn_mode is "counterclockwise"):
						self.steering_input.publish(UInt32(1050))
					else:
						self.steering_input.publish(UInt32(1950))

					self.velocity_input.publish(UInt32(turn_speed)) 
				else:
					rospy.loginfo('WAITING FOR TURN')	

			elif self.is_ball_detected:
				rospy.loginfo('AVOIDING BALL!!')
				error = self.ball_position_to_error(self.ball_position)
				#self.steering_pid.set_point = error
				#control = self.steering_pid.update()
				control = error
				PWM = control
				rospy.loginfo('command sent: {}'.format(PWM))
				self.steering_input.publish(UInt32(PWM))

			else:
				rospy.loginfo('depth_error: {}'.format((self.depth_error*1000 + 1000)))
				#rospy.loginfo('convergence_error: {}'.format((self.convergence_error*1000 + 1000)))
				normalized_error = self.depth_error
				#self.steering_pid.set_point = (normalized_error*1000.0 + 1000.0)
				#control = self.steering_pid.update()
				control = (normalized_error*1000.0 + 1000.0)
				error_temp = control - 1500.0
				error_temp *= 1.35
				control = np.clip(1500 + error_temp,1000.0,2000.0)
				rospy.loginfo('controller output: {}'.format(control))
				self.last_input = control
				PWM = control
				rospy.loginfo('command sent: {}'.format(PWM))
				self.steering_input.publish(UInt32(PWM))
				#rospy.loginfo('steering input: {}'.format(PWM))

				self.output_data.append(PWM)
				self.input_data.append(normalized_error*1000.0 + 1000.0)
				self.time_array.append(rospy.Time.now().to_sec())
			
			if (self.stop_sign):
				rospy.loginfo('STOP SIGN')
				self.velocity_input.publish(UInt32(1500))
			elif ((time.time() - self.controller_time) > 1.0):
				self.velocity_input.publish(UInt32(1500))
			else:
				rospy.loginfo('FORWARD!!')
				self.velocity_input.publish(UInt32(straight_speed))

		self.imu_turn.publish(Bool(self.imu_turn_value))

		key = self.getKey(self.settings, self.key_timeout)
		if (key == 'b'):
			rospy.loginfo('STOP was pressed')
			return True
		else:
			return False
			

	def depth_callback(self, data):
		self.depth_error = data.data
		self.controller_time = time.time()

	def convergence_callback(self, data):
		self.convergence_error = data.data

	def ball_detection_callback(self, data):
		self.is_ball_detected = data.data

	def ball_position_callback(self, data):
		self.ball_position = data.data

	def ir_sensor_callback(self, data):
		if ((data.data < 200) and (data.data > 0)):
			self.turn = True
		else:
			self.turn = False
		
	def stop_sign_callback(self, data):
		if (data.data):
			self.stop_sign = True
		else:
			self.stop_sign = False

	def crash_callback(self, data):
		if (data.data):
			self.crash = True
		else:
			self.crash = False

	def getKey(self, settings, timeout):
		tty.setraw(sys.stdin.fileno())
		# sys.stdin.read() returns a string on Linux
		rlist, _, _ = select([sys.stdin], [], [], timeout)
		if rlist:
			key = sys.stdin.read(1)
		else:
			key = ''
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
		return key
	
	def saveTerminalSettings(self):
		return termios.tcgetattr(sys.stdin)
	
	def ball_position_to_error(self, position):
		if (position > 0.5):
			error = position - 0.5
		elif (position < 0.5):
			error = position + 0.5
		else:
			error = 0.0

		return error
	
	def imu_callback(self, data):
		_,_,self.yaw = self.quaternion_to_euler(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)

	def depth_turn_callback(self, data):
		self.depth_turn_value = data.data
	
	def save_plot(self):
		# plot the data
		plt.plot(self.time_array, self.input_data)
		plt.plot(self.time_array, self.output_data)
		plt.xlabel('Time (s)')
		plt.ylabel('IR Data')
		plt.title('IR Raw Data')

		# save the plot as a .png file
		plt.savefig('PID.png')

	def quaternion_to_euler(self, x, y, z, w):
		# roll (x-axis rotation)
		sinr_cosp = 2 * (w * x + y * z)
		cosr_cosp = 1 - 2 * (x * x + y * y)
		roll = math.atan2(sinr_cosp, cosr_cosp)

		# pitch (y-axis rotation)
		sinp = 2 * (w * y - z * x)
		if abs(sinp) >= 1:
			pitch = math.copysign(math.pi / 2, sinp) # use 90 degrees if out of range
		else:
			pitch = math.asin(sinp)

		# yaw (z-axis rotation)
		siny_cosp = 2 * (w * z + x * y)
		cosy_cosp = 1 - 2 * (y * y + z * z)
		yaw = math.atan2(siny_cosp, cosy_cosp)

		return roll, pitch, yaw
	
	def is_turn_finsihed(self):
		# account for any angle wraparound
		if self.current_yaw - self.previous_yaw > math.pi:
			self.previous_yaw += 2 * math.pi
		elif self.previous_yaw - self.current_yaw > math.pi:
			self.previous_yaw -= 2 * math.pi

		# calculate the change in yaw and see if it's 45 degrees clockwise
		delta_yaw = self.current_yaw - self.previous_yaw
		rospy.loginfo('delta yaw: {}'.format(delta_yaw))
		if (turn_mode is "counterclockwise"):
			return delta_yaw <= math.pi/3
		else:
			return delta_yaw <= -math.pi/3

class PIDController:
	def __init__(self, kp, ki, kd, set_point, out_min, out_max):
		self.kp = kp
		self.ki = ki
		self.kd = kd
		self.set_point = set_point
		self.out_min = out_min
		self.out_max = out_max
		
		self.last_output = 0
		self.last_set_point = set_point
		self.integral_error = 0

		self.last_time = rospy.Time.now().to_sec()
		
	def update(self):
		time_now = rospy.Time.now().to_sec()
		dt = time_now - self.last_time
		self.last_time = time_now

		# Calculate error
		error = self.set_point - self.last_output
		
		# Calculate proportional term
		proportional = self.kp * error
		
		# Calculate integral term
		self.integral_error += error * dt
		integral = self.ki * self.integral_error
		
		# Calculate derivative term
		derivative = self.kd * (self.set_point - self.last_set_point) / dt
		self.last_set_point = self.set_point
		
		# Calculate PID output
		output = proportional + integral + derivative + (self.out_min+((self.out_max-self.out_min)/2))
		#output = max(min(output, self.out_max), self.out_min)  # Apply output limits
		output = np.clip(output,self.out_min,self.out_max)
		self.last_output = output
		
		return output
		 

if __name__ == '__main__':
	try:
		auto = Autonav()
		stop = False
		while (not rospy.is_shutdown()) and (stop is False):
			stop = auto.run()
		# register the save_plot function to be called when the node is killed
		atexit.register(auto.save_plot())
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
