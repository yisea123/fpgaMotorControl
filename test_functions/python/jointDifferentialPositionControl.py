from getRobotPose import getOptitrackPose

#---------------------------------------#---------------------------------------
#---------------------------------------#---------------------------------------
#Motor setup                            #---------------------------------------
#---------------------------------------#---------------------------------------
#---------------------------------------#---------------------------------------

import numpy as np
import time
import os
import socket
from copy import deepcopy

from utils.motor_class import motors
from utils.tcp_class import tcp_communication

joint_motor_indexes = np.array([0,1,2,3])
motor_command = np.zeros(8)
encoder_counts = 1440
gear_ratio = 470
counts_per_revolution = gear_ratio * encoder_counts
counts_per_radian = counts_per_revolution / (2 * np.pi)
counts_per_degree = counts_per_revolution / 360
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


#---------------------------------------#---------------------------------------
#---------------------------------------#---------------------------------------
#Optitrack setup                        #---------------------------------------
#---------------------------------------#---------------------------------------
#---------------------------------------#---------------------------------------
import signal
import sys
import transforms3d as tf3d

from utils.GetJointData import data, NatNetFuncs#receiveNewFrame, receiveRigidBodyFrameList
from utils.NatNetClient2 import NatNetClient

server_ip = "192.168.1.27"
multicastAddress = "239.255.42.99"
print_trak_data = False

joint_names = ['base', 'j2', 'j3', 'j4']
ids = [0, 1, 2, 3]

#Tracking class
print("Starting streaming client now...")
streamingClient = NatNetClient(server_ip, multicastAddress, verbose = print_trak_data)
NatNet = NatNetFuncs()
streamingClient.newFrameListener = NatNet.receiveNewFrame
streamingClient.rigidBodyListListener = NatNet.receiveRigidBodyFrameList
prev_frame = 0
time.sleep(0.5)
streamingClient.run()
time.sleep(0.5)
track_data = data(joint_names,ids)
track_data.parse_data(NatNet.joint_data, NatNet.frame) #updates the frame and data that is being used
#debug values
print_cartesian = True

#Control code
print("Arming motors now...")
motors.arm()
time.sleep(2)
error_cum = 100 #initialize break error to a large value

zero_position = deepcopy(motors.motor_pos)
motor_command = deepcopy(zero_position)
#---------------------------------------#---------------------------------------
#Zeroing loop                           #---------------------------------------
#---------------------------------------#---------------------------------------
j1_angle, j2_angle, j3_angle, j4_pos, joint4_base, j4b_pos, j4b_euler = getOptitrackPose(track_data, NatNet)

while np.abs(j1_angle) > 1 or np.abs(j2_angle) > 1 or np.abs(j3_angle) > 1 or np.abs(j4_pos - 65) > 1:


	j1_angle, j2_angle, j3_angle, j4_pos, joint4_base, j4b_pos, j4b_euler = getOptitrackPose(track_data, NatNet)

	k = 0.025
	kl = 0.1
	if np.abs(j1_angle) > 1:
		motor_command[joint_motor_indexes[0]] += -1* j1_angle * counts_per_degree * k
	elif np.abs(j2_angle) > 1:
		motor_command[joint_motor_indexes[1]] += j2_angle * counts_per_degree * k
	elif np.abs(j3_angle) > 1:
		motor_command[joint_motor_indexes[2]] += j3_angle * counts_per_degree * k
	elif np.abs(j4_pos - 65) > 1:
		motor_command[joint_motor_indexes[3]] += -1* (j4_pos-65) * counts_per_degree * kl
	error_cum = np.abs(j1_angle) + np.abs(j2_angle) + np.abs(j3_angle)

	print("Current joint positions: \n j1: {}\n j2: {}\n j3: {}\n j4: {}\n".format(j1_angle, j2_angle, j3_angle, j4_pos))
	print("Motor command: {}".format(motor_command))

	motors.command_motors(motor_command)

	time.sleep(0.05)

zero_position = deepcopy(motor_command)


#---------------------------------------#---------------------------------------
#---------------------------------------#---------------------------------------
#Mixing setup                           #---------------------------------------
#---------------------------------------#---------------------------------------
#---------------------------------------#---------------------------------------

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


#---------------------------------------#---------------------------------------
#---------------------------------------#---------------------------------------
#Robot control                          #---------------------------------------
#---------------------------------------#---------------------------------------
#---------------------------------------#---------------------------------------

start_time = time.time()

for i in range(int(10/dt)):

	current_time = time.time() - start_time


	value = np.sin(current_time*(2*np.pi) / 8) * (2*np.pi) / 16# * counts_per_revolution / 10
	armTheta = np.zeros((4,1))
	armTheta[0] = value
	armTheta[1] = 0
	armTheta[2] = 0
	armTheta[3] = 0

	j1_angle, j2_angle, j3_angle, j4_pos, joint4_base, j4b_pos, j4b_euler = getOptitrackPose(track_data, NatNet)

	motorSetpoint = np.dot(armTheta_motorTheta, armTheta)

	motor_command = np.zeros(8)
	motor_command[joint_motor_indexes[0]] = motorSetpoint[0] * counts_per_radian
	motor_command[joint_motor_indexes[1]] = motorSetpoint[1] * counts_per_radian * -1 # -1 to align motor axis with tracker
	motor_command[joint_motor_indexes[2]] = motorSetpoint[2] * counts_per_radian * -1 # -1 to align motor axis with tracker
	motor_command[joint_motor_indexes[3]] = motorSetpoint[3] * counts_per_radian # -1 to align motor axis with tracker
	motor_command += zero_position
	print("arm degrees: {}".format(armTheta * 180 / np.pi))
	print("motor degrees: {}".format(motorSetpoint * 180 / np.pi))

	motors.command_motors(motor_command)

	time.sleep(dt) 
#---------------------------------------#---------------------------------------
#---------------------------------------#---------------------------------------
#Shutdown                               #---------------------------------------
#---------------------------------------#---------------------------------------
#---------------------------------------#---------------------------------------

data = ('b'+ 'stop' +'d')
my_socket.send(data.encode())
my_socket.shutdown(socket.SHUT_RDWR)
my_socket.close()
streamingClient.stop()
print("\n\nFinished zeroing, closing ports and exiting now\n")
sys.exit(0)
