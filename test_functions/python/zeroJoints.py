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

joint_motor_indexes = np.array([4,1,6,5])
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
socket_port = 1121
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

while error_cum > 5:


	#---------------------------------------#---------------------------------------
	#Get Optitrack data                     #---------------------------------------
	#---------------------------------------#---------------------------------------
	track_data.parse_data(NatNet.joint_data, NatNet.frame) #updates the frame and data that is being used
	#print(track_data.frame)

	if print_cartesian:
		base = track_data.bodies[0].position
		j1 = track_data.bodies[1].position
		j2 = track_data.bodies[2].position
		j3 = track_data.bodies[3].position
		print("joint positions in camera frame\n base: {0[0]:.4f}, {0[1]:.4f}, {0[2]:.4f}\n j1: {1[0]:.4f}, {1[1]:.4f}, {1[2]:.4f}\n j2: {2[0]:.4f}, {2[1]:.4f}, {2[2]:.4f}\n j3: {3[0]:.4f}, {3[1]:.4f}, {3[2]:.4f}\n".format(base, j1, j2, j3))
	
	base = track_data.bodies[0].homogenous_mat
	base_inv = track_data.bodies[0].homg_inv
	joint2 = track_data.bodies[1].homogenous_mat
	joint2_inv = track_data.bodies[1].homg_inv
	joint3 = track_data.bodies[2].homogenous_mat
	joint3_inv = track_data.bodies[2].homg_inv
	joint4 = track_data.bodies[3].homogenous_mat
	joint4_inv = track_data.bodies[3].homg_inv

	joint2_base, j2b_pos, j2b_euler, _ = track_data.homg_mat_mult(base_inv,joint2) #joint2 in base frame -> moves only in base Y+X axis
	joint3_joint2, j3j2_pos, j3j2_euler, _ = track_data.homg_mat_mult(joint2_inv,joint3)
	joint4_joint3, j4j3_pos, j4j3_euler, _ = track_data.homg_mat_mult(joint3_inv,joint4)


	j2b_deg = np.array(j2b_euler) * 180 / np.pi
	j2b_pos_mm = np.array(j2b_pos)*1000
	j3j2_deg = np.array(j3j2_euler) * 180 / np.pi
	j3j2_pos_mm = np.array(j3j2_pos)*1000
	j4j3_deg = np.array(j4j3_euler) * 180 / np.pi
	j4j3_pos_mm = np.array(j4j3_pos)*1000


	j1_angle = j2b_deg[0]
	j2_angle = j2b_deg[1]
	j3_angle = j3j2_deg[1]
	j4_pos = j4j3_pos_mm[2]


	#joint 1 optitrack + == motor +
	#joint 2 optitrack + == motor +
	#joint 3 optitrack + == motor -
	#joint 4 optitrack + == motor +

	k = 0.025
	if np.abs(j1_angle) > 1:
		motor_command[joint_motor_indexes[0]] += -1* j1_angle * counts_per_degree * k
	elif np.abs(j2_angle) > 1:
		motor_command[joint_motor_indexes[1]] += j2_angle * counts_per_degree * k
	elif np.abs(j3_angle) > 1:
		motor_command[joint_motor_indexes[2]] += j3_angle * counts_per_degree * k

	error_cum = np.abs(j1_angle) + np.abs(j2_angle) + np.abs(j3_angle)

	print("Current joint positions: \n j1: {}\n j2: {}\n j3: {}\n j4: {}\n".format(j1_angle, j2_angle, j3_angle, j4_pos))
	print("Motor command: {}".format(motor_command))

	motors.command_motors(motor_command)

	time.sleep(0.05)

data = ('b'+ 'stop' +'d')
my_socket.send(data.encode())
my_socket.shutdown(socket.SHUT_RDWR)
my_socket.close()
streamingClient.stop()
print("\n\nFinished zeroing, closing ports and exiting now\n")
sys.exit(0)
