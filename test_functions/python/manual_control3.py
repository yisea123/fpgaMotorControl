import numpy as np
import os
import time
import socket
import re
import sys
import getch
import select
import signal
import numpy as np
import matplotlib.pyplot as plt

#grab encoder counts and limit switch values

def signal_handler(signal, frame):
	data = ('b'+ 'stop' +'d')
	CLIENT_SOCKET.send(data.encode())
	CLIENT_SOCKET.shutdown(socket.SHUT_RDWR)
	CLIENT_SOCKET.close()
	print("\n\nCtrl z pressed, closing ports and exiting now\n")
	sys.exit(0)

class tcp_communication():
	def __init__(self, ip, port):

		self.ip = ip
		self.port = port

	def open_socket(self):
		#Initialize socket
		print("Opening socket at ip: {} using port: {}".format(self.ip, self.port))
		CLIENT_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		CLIENT_SOCKET.connect((self.ip, self.port))
		CLIENT_SOCKET.setblocking(1)
		#CLIENT_SOCKET.settimeout(1) #if setblocking is 1, this is basically settimeout(None)

		# #Run a single command to initalize communication
		# self.command_motors(position)
		# time.sleep(1)
		return CLIENT_SOCKET

	def setpriority(self, pid=None, priority=5):
	    """ Set The Priority of a Windows Process.  Priority is a value between 0-5 where
	        2 is normal priority.  Default sets the priority of the current
	        python process but can take any valid process ID. """
	        
	    import win32api,win32process,win32con
	    
	    priorityclasses = [win32process.IDLE_PRIORITY_CLASS,
	                       win32process.BELOW_NORMAL_PRIORITY_CLASS,
	                       win32process.NORMAL_PRIORITY_CLASS,
	                       win32process.ABOVE_NORMAL_PRIORITY_CLASS,
	                       win32process.HIGH_PRIORITY_CLASS,
	                       win32process.REALTIME_PRIORITY_CLASS]
	    if pid == None:
	        pid = win32api.GetCurrentProcessId()
	    handle = win32api.OpenProcess(win32con.PROCESS_ALL_ACCESS, True, pid)
	    win32process.SetPriorityClass(handle, priorityclasses[priority])

class motors():
	def __init__(self, CLIENT_SOCKET, dt, position, step_size, degrees_count_motor):

		#Declaring class variables
		self.dt = dt
		self.motor_pos = position
		self.step_size = step_size
		self.degrees_count_motor = degrees_count_motor
		self.sine_speed = 0.3
		self.sine_travel = 3
		self.motor_nums = "12345678"

		self.encoder_positions_list = []

		self.motor_encoders_data = np.zeros(8)
		self.joint_encoders_data = np.zeros(4)
		self.limit_data = np.zeros(8)
		self.client_socket = CLIENT_SOCKET

	def command_motors(self, pos):
		self.motor_pos = pos
		data = ('b'+ str(int(pos[0])) + ' ' + str(int(pos[1])) + ' ' + str(int(pos[2])) + ' ' +
					 str(int(pos[3])) + ' ' + str(int(pos[4])) + ' ' + str(int(pos[5])) + ' ' +
					 str(int(pos[6])) + ' ' + str(int(pos[7])) +  ' ' + str(int(pos[8])) + 'd')
		self.client_socket.send(data.encode())
		self.read_buff()
		return self.motor_pos

	def read_buff(self):
	 	data = str(CLIENT_SOCKET.recv(256))
	 	data = re.split('\s', data)
	 	print("this is the data read", data)
	 	if data[1] == 'closeports':
	 		self.client_socket.close()
	 		print("\n\nServer side closed. Closing ports now.\n\n")
	 		sys.exit()

	 	self.motor_encoders_data = np.array(list(map(int, data[1:9])))
	 	self.joint_encoders_data = np.array(list(map(int, data[17:21])))
	 	self.limit_data = np.array(list(map(int, data[9:17])))
	 	if PRINT_SENSORS:
	 		print('Motor encoder positions {}'.format(self.motor_encoders_data))
	 		print('Joint encoder positions {}'.format(self.joint_encoders_data))
	 		print('Limit switch values {}'.format(self.limit_data))

	 	self.encoder_positions_list.append(int(data[5]))
	 	return

	def which_motors(self):
		""" Returns a string of numbers which represents which motors to control. """
		self.motor_nums = input("Enter motor numbers to control(1-8, no spaces or commas ex: 158), to control all press enter: ")
		if len(self.motor_nums) == 0:
			self.motor_nums = "12345678" #String!
		return self.motor_nums

	def zero_motors(self):
		zero_pos = np.zeros((9,1)).astype(int)
		self.command_motors(zero_pos)
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
		start_pos = self.motor_pos
		current_pos = np.insert(1,1,np.zeros(8))
		time_diff = []
		counter = 1

		print("Running sine wave, ctrl c to break:")
		print('current positions {}' .format(self.motor_pos[1:]))

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

				for i in range(len(current_pos)):
					if i==0:
						current_pos[i] = 1;
					else:
						current_pos_1 = (np.sin((current_time-start_time)*2*np.pi*self.sine_speed)/self.degrees_count_motor*360/16*self.sine_travel)
						current_pos[i] = start_pos[i] + current_pos_1

				self.command_motors(current_pos)

				counter = counter + 1

				if counter%10 == 0 and PRINT_LOOP_SPECS == 1:
					print("Mean loop time is: {}".format(np.mean(np.asarray(time_diff))))
					print("Loop time variance is: {}".format(np.var(np.asarray(time_diff))))
					counter = 1

				if len(time_diff) > 1e4:
					time_diff = time_diff[-10000:]

				if counter%1000 == 0 and False:
					plt.plot(self.encoder_positions_list)

					#if counter == 50:
					plt.show()

				#print(current_pos)
				#time.sleep(self.dt)
		except KeyboardInterrupt:
			#Gives back control to all motors
			self.motor_nums = "12345678"
			self.motor_pos = current_pos
			return self.motor_nums
		return

	def get_direction(self):
		while True:
			direction = input("Enter command or direction +/- keys and number of counts(ex:+ 100), 'p' to print command menu or ctrl z to end: ")
			direction = re.split('\s',direction)

			#Single string input with no spaces
			if len(direction) == 1:

				#If no number is given uses most recent step_size to update position
				if direction[0] == '':
					return self.step_size
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

#**********************************************************************
#**************					Begin Control		 	***************
#**********************************************************************

#Flags
PRINT_SENSORS = 1 						#Encoder positions
PRINT_LOOP_SPECS = 0 					#Loop speeds, mean time and variance
IsWindows = os.name == 'nt' 			#os.name is name of os, ex: linux='posix', windows='nt', mac='mac'
ManualControl = 1						#main loop control

#Constants and initializations
socket_ip = '192.168.1.14'
socket_port = 1114
degrees_count_motor = 0.00743			#Motor encoder resolution
degrees_count_joint = 360/1440			#Joint encoder resolution
step_size = 100							#Number of encoder counts to step
dt = 1/200
start_time = time.time()
counter = 0
position = np.insert(1,1,np.zeros(8))


#Signal handler
signal.signal(signal.SIGTSTP, signal_handler)

#Communication class
tcp = tcp_communication(socket_ip, socket_port)
CLIENT_SOCKET = tcp.open_socket()
if IsWindows:
	tcp.setpriority()

#Motor class
motors = motors(CLIENT_SOCKET, dt, position, step_size, degrees_count_motor)
motors.read_buff()
position = position + np.insert(motors.motor_encoders_data,0,0) #Adding in offset from values stored in text file onboard the fpga
motors.command_motors(position) #initialize to encoder positions

#tells which motors to control
motor_numbers = motors.which_motors()


while(ManualControl):

	""" Blocking function, main call for functionality.
		Returns either a array of positions or a string 
		indicating what action was done """
	command = motors.get_direction()

	#Checks if command is an array of positions or string command
	if isinstance(command, str):
		#Does nothing to motors here -> control stuff is done in functions
		if command == "z":
			position = np.insert(1,1,np.zeros(8))
			#Gives motors length 0
			motor_numbers = ""
			command = 0
			continue

		elif command == "sine":
			position = motors.motor_pos
			motor_numbers = motors.motor_nums
			command = 0

		else:
			motor_numbers = command
			command = 0

	for i in range(len(motor_numbers)):
 		position[int(motor_numbers[i])] = position[int(motor_numbers[i])] + command

	print(motor_numbers)
	print(position)
	motors.command_motors(position)
	time.sleep(motors.dt)

#**********************************************************************
#**************					End Control		 	    ***************
#**********************************************************************


























'''

#setpriority()
x = np.ones(9).astype(int)
x[1:] = x[1:] + 5
command_motors(client_socket, x)
time.sleep(2)

j = 0
encoder_position = 0
error = 0
P=300
#P=0
x_temp = np.ones(9)

encoder_steps = np.arange(0,250)
counter = 0
start_time = time.time()

degrees_per_count_motor = 0.00743
degrees_per_count_joint = 360 / 1440

while (1): #time.time()-start_time<25):

	encoder_positions = str(client_socket.recv(256))
	encoders = re.split('\s', encoder_positions)

	if counter % 50 == 0:
		#print("Encoder1: ", encoders[1],"Encoder2: ", encoders[2],"Encoder3: ", encoders[3],"Encoder4: ", encoders[4])
		#print("Encoder Setpoint: ", encoder_position)
		#print("Error: ", error)
		#print("Temp: ",x_temp)
		print("output error is {}, encoder joint is {}".format(error, float(encoders[1]) * degrees_per_count_joint))


	tst = (time.time()-start_time)
	encoder_position = (np.sin(tst*0.5)*40/degrees_per_count_motor)*1


	error = encoder_position * degrees_per_count_motor - float(encoders[1]) * degrees_per_count_joint

	x_temp[3] = (encoder_position + error*P).astype(int)
	x_temp[4] = (-encoder_position - error*P).astype(int)
	x_temp[1] = x_temp[3]
	x_temp[2] = x_temp[4]

	#x[1] = np.round(x[1] + error*P).astype(int)
	#x[2] = np.round(x[2] - error*P).astype(int)

	# x[1:5] = np.round(x[1:5] + error*P).astype(int)
	# x[5:9] = np.round(x[5:9] - error*P).astype(int)

	counter = counter + 1
	command_motors(client_socket,x_temp)
	time.sleep(dt)


	#current_pos_1 = (np.sin((time.time()-start_time)*2)*1000)*(1)
			
	# for i in range(len(current_pos)):
	# 	if i==0:
	# 		current_pos[i] = 1;
	# 	else:
	# 		current_pos[i] = np.power(-1,(i+1))*current_pos_1
	# command_motors(client_socket, current_pos)
	# time.sleep(dt)
	
	#if np.abs(current_pos_1) < 5:
	#	time.sleep(0)
	
	# if j%50 == 0:
	# 	print('zeros: ' + str(current_pos))
	# j = j+1
	









for i in range(len(current_pos)):
	current_pos = np.zeros(9)
	command_motors(client_socket, current_pos)
'''
