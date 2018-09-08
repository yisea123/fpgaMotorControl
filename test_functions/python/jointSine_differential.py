import numpy as np
import time
import os
import socket
from copy import deepcopy

from utils.motor_class import motors
from utils.tcp_class import tcp_communication

joint_motor_indexes = np.array([4,1,6,5])


#---------------------------------------#---------------------------------------
#Constants and definition declarations  #---------------------------------------
#---------------------------------------#---------------------------------------
#Flags
PRINT_SENSORS = 0						#Encoder positions
PRINT_LOOP_SPECS = 1 					#Loop speeds, mean time and variance
DISABLED = True							#Motor controller disabled on start
IsWindows = os.name == 'nt' 			#os.name is name of os, ex: linux='posix', windows='nt', mac='mac'
ManualControl = 1						#main loop control
OPPOSITE_SINE = True

#Constants and initializations for tcp and motor class
socket_ip = '192.168.1.39'
socket_port = 1122
dt = 0.01 #100 hz
start_time = time.time()
counter = 0

encoder_counts = 1440
gear_ratio = 470
counts_per_revolution = gear_ratio * encoder_counts
counts_per_radian = counts_per_revolution / (2 * np.pi)


#Communication class
tcp = tcp_communication(socket_ip, socket_port)
my_socket = tcp.open_socket()
if IsWindows:
	tcp.setpriority()

#Motor class
motors = motors(CLIENT_SOCKET = my_socket, dt = dt, step_size = 100, degrees_count_motor = 1./counts_per_revolution, degrees_count_motor_joint = 1, PRINT_SENSORS = PRINT_SENSORS)
motors.read_buff() 						#c side sends out initial stored encoder positions, grabs them
print(motors.motor_encoders_data)
time.sleep(1)
motors.read_buff()
print("initializing motors to {}".format(motors.motor_encoders_data))
motors.motor_pos = motors.motor_encoders_data 	#initialize to stored encoder positions
start_pos = deepcopy(motors.motor_pos)
motors.command_motors(motors.motor_pos)			#echo read value, values arent used since c side is in err mode

zero_value = motors.motor_pos.copy()
#Motor arm mixing
Db = 16.5
Ds = 8.72
Dm = 13.88
Dl = 18
Dj1 = 21
Dj2 = 21.5
Dj3 = 13.5

motorTheta_armTheta = np.eye(4) * np.array([Db/Dj1, Db/Dj2, Db/Dj3, Db])
motorTheta_armTheta = np.dot(np.array([[1, 0, 0, 0], [Dl/Dj2, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]), motorTheta_armTheta)
motorTheta_armTheta = np.dot(np.array([[1, 0, 0, 0], [0, 1, 0, 0], [Dm/Dj3, -Dm/Dj3, 1, 0], [0, 0, 0, 1]]), motorTheta_armTheta)
motorTheta_armTheta = np.dot(np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [-Ds, Ds, Dl, 1]]), motorTheta_armTheta)


armTheta_motorTheta = np.linalg.inv(motorTheta_armTheta)

#linear is in mm/radian, rotary is unitless

print(motorTheta_armTheta)
print(armTheta_motorTheta)


#Control code
print("Arming motors now...")
motors.arm()
time.sleep(2)

start_time = time.time()

for i in range(int(10/dt)):

	current_time = time.time() - start_time

	#for i in range(len(motors.motor_pos)):
	#	motors.motor_pos[i] = 0

	value = np.sin(current_time*(2*np.pi) / 8) * (2*np.pi) / 16# * counts_per_revolution / 10
	armTheta = np.zeros((4,1))
	armTheta[0] = value
	armTheta[1] = value
	armTheta[2] = value
	#armTheta = np.ones(4) * np.pi/4
	motorSetpoint = np.dot(armTheta_motorTheta, armTheta)

	motor_command = np.zeros(8)
	motor_command[joint_motor_indexes[0]] = motorSetpoint[0] * counts_per_radian
	motor_command[joint_motor_indexes[1]] = motorSetpoint[1] * counts_per_radian * -1 # -1 to align motor axis with tracker
	motor_command[joint_motor_indexes[2]] = motorSetpoint[2] * counts_per_radian * -1 # -1 to align motor axis with tracker
	motor_command[joint_motor_indexes[3]] = motorSetpoint[3] * counts_per_radian
	motor_command += zero_value
	print("arm degrees: {}".format(armTheta * 180 / np.pi))
	print("motor degrees: {}".format(motorSetpoint * 180 / np.pi))

	motors.command_motors(motor_command)

	time.sleep(dt) 

data = ('b'+ 'stop' +'d')
my_socket.send(data.encode())
my_socket.shutdown(socket.SHUT_RDWR)
my_socket.close()
print("\n\nFinished test, closing ports and exiting now\n")