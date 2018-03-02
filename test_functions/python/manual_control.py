#importing some things
import numpy as np
import os
import time
import socket
import re

client_socket = 1
if(client_socket):
	client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	client_socket.connect(('192.168.1.10', 1114))
	client_socket.setblocking(0)
	client_socket.settimeout(0.1)

cpr = 500
current_pos_1 = 0
current_pos_2 = 0
base_vel = 1500
dt = 1/500

def setpriority(pid=None,priority=5):
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

def command_motors(client_socket, pos):
	data = 'b'+str(int(pos[0])) + ' ' + str(int(pos[1])) + ' ' + str(int(pos[2])) + ' ' + str (int(pos[3])) + ' ' + str (int(pos[4]))  + ' ' + str (int(pos[5])) + ' ' + str (int(pos[6])) + ' ' + str (int(pos[7])) +  ' ' + str(int(pos[8])) + 'd'
	client_socket.send(data.encode())
	return data

def incr_pos(position_increment, current_pos):
	return current_pos + position_increment
	


#Read from user what motors to control
def which_motors():
	motors = input("Enter motor numbers to control(1-8, no spaces or commas ex: 158), to control all press enter: ")
	if len(motors) == 0:
		motors = "12345678"
	return motors

def zero_motors(client_socket):
	zero_pos = np.zeros((9,1)).astype(int)
	command_motors(client_socket, zero_pos)
	print("System Zeroing")
	time.sleep(3)
	return

# + is out and - is in
def get_direction(default_counts,client_socket):
	while True:
		direction = input("Enter direction +/- keys and number of counts(ex:+ 100): ")
		direction = re.split('\s',direction)

		#If no number is given uses default counts
		if len(direction) == 1:
			if direction[0] == '':
				counts = default_counts
				return counts
				break

			if direction[0] == "m":
				counts = which_motors()
				return counts
				break

			if direction[0] == "z":
				zero_motors(client_socket)
				return "z"
				break

			direction.append(str(default_counts))

			

		if len(direction)>2 or not(direction[0] == '+' or '-' or not(direction[1].isdigit())):
			print("Incorrect input, try again.")
			continue
		else:
			if direction[0] == '-':
				counts = -abs(int(direction[1]))
			else:
				counts = abs(int(direction[1]))

			return counts
			break
	return;




#**********************************************************************
#**************					Manual control		 	***************
#**********************************************************************

#Constants
manual_control = 1
degrees_count_motor = 0.00743
degrees_count_joint = 360/1440
default_counts = 100
counter = 0
start_time = time.time()
loop_time = 1/500

#initialization
x = np.ones(9).astype(int)
command_motors(client_socket,x)
time.sleep(2)

#Function to ask which motors to control
motors = which_motors()

while(manual_control):
	#Set default counts to last input
	encoder_positions = str(client_socket.recv(256))
	encoders = re.split('\s', encoder_positions)

	#Blocking function
	default_counts = get_direction(default_counts,client_socket)	

	#If return (default_counts) is string indicating which motors to control
	if isinstance(default_counts, str):
		if default_counts == "z":
			x = np.ones(9).astype(int)
			#Gives motors length 0
			motors = ""
			default_counts = 0

		else:
			motors = default_counts
			default_counts = 0

	for i in range(len(motors)):
 		x[int(motors[i])] = x[int(motors[i])] + default_counts

	#print(default_counts)
	print(motors)
	print(x)
	#print(encoders)
	command_motors(client_socket,x)
	time.sleep(loop_time)

#**********************************************************************
#**************				Done Manual control		 	***************
#**********************************************************************
























#**********************************************************************
#**************					Zeroing				   	***************
#**********************************************************************
# zeroing = 0

# if zeroing == 1:
# 	current_pos = np.zeros((9,1))

# 	command_motors(client_socket, current_pos)

# 	#to enable zeroing set state to 0
# 	state = 0
# 	client_socket.setblocking(1)
# 	while(state==0):
# 			switch_state = str(client_socket.recv(256))
# 			test = re.split('\s', switch_state)
# 			print("Current state is: ", test[1],"rate is: ", test[2])
# 			state=int(test[1])
# 			command_motors(client_socket, current_pos)
# 			time.sleep(dt)

# 	time.sleep(5)

# 	x = np.ones(9).astype(int)
# 	x[1:] = (x[1:]-5000).astype(int)
# 	print(x)
# 	command_motors(client_socket, x)
# 	time.sleep(2)
# 	x[1:] = (x[1:]-5000).astype(int)
# 	print(x)
# 	command_motors(client_socket, x)
# 	time.sleep(2)
# 	x[1:] = (x[1:]-5000).astype(int)
# 	print(x)
# 	command_motors(client_socket, x)
# 	time.sleep(2)

# 	print("Finishing zeroing")
# 	print("Current state is: ", test[1],"rate is: ", test[2])

#**********************************************************************
#**************					Done Zeroing		 	***************
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