import numpy as np
import os
import time
import socket
import re
import sys
import select
import signal
import numpy as np
import matplotlib.pyplot as plt
from copy import deepcopy


class motors():
	def __init__(self, CLIENT_SOCKET, dt, step_size, degrees_count_motor, degrees_count_motor_joint, PRINT_SENSORS, results_dir = None):

		#Declaring class variables
		self.dt = dt 													#time step
		self.motor_pos = np.zeros(8) 									#motor position in encoder counts, values sent
		self.step_size = step_size										#number of encoder counts to step by
		self.degrees_count_motor = degrees_count_motor 					#degrees of motor shaft rotation per encoder count
		self.degrees_count_motor_joint = degrees_count_motor_joint		#degrees or joint roation per encoder count
		self.sine_speed = 0.05											#frequency Hz
		self.sine_travel = 0.4
		self.motor_revolutions = 1.0									#number of rotations to make in profile, remember it'll be +- full rotations since sine
		self.motor_nums = "12345678" 									#which motor to control on the bank, number corresponds to which motor 1-8
		self.results_dir = results_dir									#Directory to where motor data is stored

		#Linear encoder controller parameters
		self.setpoint = 0
		self.p_gain = 10
		self.i_gain = -0.25
		self.d_gain = 0

		self.profile_time = []
		self.profile_values_sent = []
		self.profile_values_recv = []

		self.motor_encoders_data = np.zeros(8)
		self.joint_encoders_data = np.zeros(4)
		self.limit_data = np.zeros(8)
		self.client_socket = CLIENT_SOCKET
		self.PRINT_SENSORS = PRINT_SENSORS

	def command_motors(self, pos): #sends new positions to controller and updates local position with encoder reads
		#self.motor_pos = pos
		data = ('b'+ str(int(pos[0])) + ' ' + str(int(pos[1])) + ' ' + str(int(pos[2])) + ' ' +
					 str(int(pos[3])) + ' ' + str(int(pos[4])) + ' ' + str(int(pos[5])) + ' ' +
					 str(int(pos[6])) + ' ' + str(int(pos[7])) + 'd')
		self.client_socket.send(data.encode())
		self.read_buff()
		return self.motor_pos

	def read_buff(self):
		global DISABLED
		data = str(self.client_socket.recv(256))
		data = re.split('\s', data)
		if data[1] == 'closeports':
			self.client_socket.close()
			print("\n\nServer side closed. Closing ports now.\n\n")
			sys.exit()

		if data[1] == 'err':
			DISABLED = True
			print("*** C side has an error or needs to be armed ***\n")

		else:
			self.motor_encoders_data = np.array(list(map(int, data[1:9])))
			self.joint_encoders_data = np.array(list(map(int, data[17:21])))
			self.limit_data = np.array(list(map(int, data[9:17])))
			if self.PRINT_SENSORS:
				print('Read motor encoder positions {}'.format(self.motor_encoders_data))
				print('Read joint encoder positions {}'.format(self.joint_encoders_data))
				print('Read limit switch values {}'.format(self.limit_data))
		return

	def arm(self):
		global DISABLED
		data = ('b' + 'arm' + 'd')
		DISABLED = 0
		self.client_socket.send(data.encode())
		self.read_buff()
		return

	def which_motors(self):
		""" Returns a string of numbers which represents which motors to control. """
		self.motor_nums = input("Enter motor numbers to control(1-8, no spaces or commas ex: 158), to control all press enter: ")
		if len(self.motor_nums) == 0:
			self.motor_nums = "12345678" #String!
		return self.motor_nums

	def zero_motors(self):
		data = ('b' + 'zero' + 'd')
		self.client_socket.send(data.encode())
		self.read_buff()
		self.motor_pos = deepcopy(self.motor_encoders_data)
		#zero_pos = np.zeros((9,1)).astype(int)
		#self.command_motors(zero_pos)
		print("System zeroing wait until motors stop moving...")
		time.sleep(3)
		return

	def print_menu(self):
		print('\n')
		print("enter - advances forward by last step size (initial step size if no step size indicated")
		print("m - allows to change which motors to control")
		print("z - starts the zeroing process, needs limit switches attached")
		print("sine - starts a sine wave pulse")
		print("data - prints all the sensor feedback")
		return

	# def print_data(self):
	# 	data = str(CLIENT_SOCKET.recv(256))
	# 	data = re.split('\s', data)

	# 	self.motor_encoders_data = data[1:9]
	# 	self.joint_encoders_data = data[17:21]
	# 	self.limit_data = data[9:17]
	# 	print('\n')
	# 	print('Current motor encoder positions {}'.format(self.motor_encoders_data))
	# 	print('Current joint encoder positions {}'.format(self.joint_encoders_data))
	# 	print('Current limit switch values {}'.format(self.limit_data))
	# 	return

	def run_sine(self):
		print("Running sine wave, ctrl c to break:")
		print('current positions {}' .format(self.motor_pos[1:]))
		time_diff = []
		counter = 1
		start_pos = self.motor_pos
		commanded = np.zeros(8)
		time.sleep(2)
		start_time = time.time()
		st = start_time

		time.sleep(self.dt)

		try:
			while True:
				current_time = time.time()
				ct = time.time()
				dt = ct - st
				time_diff.append(dt)
				st = ct
				#16 since 16 threads per inch
				current_pos_1 = (np.sin((current_time-start_time)*2*np.pi*self.sine_speed)/self.degrees_count_motor_joint*360/16*self.sine_travel)

				for i in range(len(commanded)):
					if OPPOSITE_SINE and i < 4:
						commanded[i] = start_pos[i] - int(current_pos_1)
					else:
						commanded[i] = start_pos[i] + int(current_pos_1)

				self.command_motors(commanded)
				counter = counter + 1
				if counter%10 == 0 and PRINT_LOOP_SPECS == 1:
					#print("Mean loop time is: {}".format(np.mean(np.asarray(time_diff))))
					#print("Loop time variance is: {}".format(np.var(np.asarray(time_diff))))
					counter = 1
				if len(time_diff) > 1e4:
					time_diff = time_diff[-10000:]
				#time.sleep(self.dt)

		except KeyboardInterrupt:
			#Gives back control to all motors
			self.motor_pos = commanded
			self.motor_nums = "12345678"
			time.sleep(self.dt)
			return self.motor_nums
		return

	def motor_profile(self): #no load
		print("\nRunning motor profile, ctrl c to break:")
		print('Starting position {}\n' .format(self.motor_pos[1:]))
		start_pos = self.motor_pos
		commanded = np.zeros(8)
		counter = 1
		time.sleep(1)
		start_time = time.time()
		time.sleep(self.dt)
		self.profile_values_sent.append(self.motor_pos[1])
		self.profile_values_recv.append(self.motor_pos[1])
		self.profile_time.append(start_time)

		try:
			while True:
				current_time = time.time()
				update_pos = np.sin(((current_time-start_time)*2*np.pi*self.sine_speed)) * 2000 * self.motor_revolutions
				for i in range(len(start_pos)):
						commanded[i] = start_pos[i] + int(update_pos)

				self.command_motors(commanded)
				self.profile_values_sent.append(self.motor_pos[1])
				self.profile_values_recv.append(self.motor_encoders_data[1])
				self.profile_time.append(current_time-start_time)
				counter = counter + 1

		except KeyboardInterrupt:
			#Gives back control to all motors
			#print(self.profile_values_sent)
			self.motor_pos = commanded
			np.savez(self.results_dir, np.array(self.profile_values_sent), np.array(self.profile_values_recv), np.array(self.profile_time))
			self.motor_nums = "12345678"
			time.sleep(self.dt)
			return self.motor_nums
		return

	def run_controller(self):
		print("\n Starting linear position control, ctrl c to break:")
		#running control on this guy -> self.joint_encoders_data[0]
		self.setpoint = self.joint_encoders_data[0]
		print(self.setpoint)

		start_pos = deepcopy(self.motor_pos)
		command = deepcopy(self.motor_pos)
		error = 0
		counter = 0
		prev_time = 0
		error_cum = 0
		

		counter = 1
		time.sleep(1)

		start_time = time.time()
		try:
			while True:
				current_time = time.time()
				error = self.setpoint - self.joint_encoders_data[0]
				error_cum += error
				command[7] = start_pos[7] + self.p_gain * error + self.i_gain * error_cum
				self.command_motors(command)

				if counter%100 == 0:
					print("starting position is: {}".format(start_pos[7]))
					print("time difference is: {}".format(current_time-prev_time))
					print("linear encoder error is {}".format(error))
					print("command sent: {}".format(command[7]))

				prev_time = current_time
				counter = counter + 1
				time.sleep(0.01)




		except KeyboardInterrupt:
			self.motor_pos = command
			self.motor_nums = "12345678"
			time.sleep(self.dt)
			return self.motor_nums


	def get_direction(self):
		while True:
			direction = input("Enter direction and counts (ex:+ 100), 'p' to print command menu, ctrl z to end and 'a' to arm: ")
			direction = re.split('\s',direction)

			#Single string input with no spaces
			if len(direction) == 1:

				#If no number is given uses most recent step_size to update position
				if direction[0] == '':
					return self.step_size
					break

				if direction[0] == 'a':
					global DISABLED
					if DISABLED:
						print("Arming now")
						self.arm()
						DISABLED = 0
					else:
						print("Already armed")
					#want same behavior as return from sine when returning control
					return "sine"
					break

				#m indicates that the user would like to change which motors to control
				if direction[0] == "m":
					self.which_motors()
					#String return
					return self.motor_nums #Returns which motors to control
					break

				if direction[0] == 'p':
					self.print_menu()

				if direction[0] == 'data':
					self.print_data()

				#z indicates to zero the motors
				if direction[0] == "z":
					self.zero_motors()
					self.step_size = 0
					#String return
					return "z"
					break

				if direction[0] == "sine":
					self.run_sine()
					#string return
					return "sine"
					break

				if direction[0] == "profile":
					self.motor_profile()
					return "sine"
					break

				#Linear control using mounted linear encoder
				if direction[0] == "ctrl":
					self.run_controller()
					return "sine"
					break

				print("Re-enter command now")
				print('\n')
				continue

			#String input of two entries (ex:+ 100)
			if len(direction)>2 or not(direction[1].isdigit()) or not(direction[0] == '+' or direction[0] == '-'):
				print("Incorrect input, try again.")
				continue
			else:
				if direction[0] == '-':
					self.step_size = -abs(int(direction[1]))
				else:
					self.step_size = abs(int(direction[1]))

				return self.step_size
				break
		return