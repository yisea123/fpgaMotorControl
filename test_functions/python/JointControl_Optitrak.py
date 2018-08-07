import numpy as np
import time
import signal
import os
import socket
import sys
import transforms3d as tf3d
from copy import deepcopy


from utils.GetJointData import data, NatNetFuncs#receiveNewFrame, receiveRigidBodyFrameList
from utils.NatNetClient2 import NatNetClient
from utils.motor_class import motors
from utils.tcp_class import tcp_communication

def signal_handler(signal, frame):
	data = ('b'+ 'stop' +'d')
	CLIENT_SOCKET.send(data.encode())
	CLIENT_SOCKET.shutdown(socket.SHUT_RDWR)
	CLIENT_SOCKET.close()
	streamingClient.stop()
	print("\n\nCtrl c pressed, closing ports and exiting now\n")
	sys.exit(0)


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
socket_ip = '192.168.1.29'
socket_port = 1115
degrees_count_motor_joint = 0.00743		#joint degrees a count
degrees_count_motor = 360/2000			#Motor encoder resolution
degrees_count_joint = 360/1440			#Joint encoder resolution
step_size = 100							#Number of encoder counts to step
dt = 1/200
start_time = time.time()
counter = 0

#opti trak values
server_ip = "192.168.1.4"
multicastAddress = "239.255.42.99"
print_trak_data = False
joint_names = ['base', 'joint1', 'joint2']
ids = [0,1,2]


#---------------------------------------#---------------------------------------
#Function calls and class declarations  #---------------------------------------
#---------------------------------------#---------------------------------------
signal.signal(signal.SIGINT, signal_handler)

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

#Communication class
tcp = tcp_communication(socket_ip, socket_port)
CLIENT_SOCKET = tcp.open_socket()
if IsWindows:
	tcp.setpriority()

#Motor class
motors = motors(CLIENT_SOCKET, dt, step_size, degrees_count_motor, degrees_count_motor_joint, PRINT_SENSORS)
motors.read_buff() 						#c side sends out initial stored encoder positions, grabs them
print(motors.motor_encoders_data)
time.sleep(1)
motors.read_buff()
print("initializing motors to {}".format(motors.motor_encoders_data))
motors.motor_pos = motors.motor_encoders_data 	#initialize to stored encoder positions
start_pos = deepcopy(motors.motor_pos)
motors.command_motors(motors.motor_pos)			#echo read value, values arent used since c side is in err mode

#---------------------------------------#---------------------------------------

print("Arming motors now...")
motors.arm()
time.sleep(1)

#classify controller later?
#set calibration phase?

#Updates values in class to latest numbers
track_data.parse_data(NatNet.joint_data, NatNet.frame) #updates the frame and data that is being used
base_inv = track_data.bodies[0].homg_inv
joint1 = track_data.bodies[1].homogenous_mat
joint1_inv = track_data.bodies[1].homg_inv
joint2 = track_data.bodies[2].homogenous_mat

#Transformation matricies
joint1_base, _, _, _ = track_data.homg_mat_mult(base_inv,joint1) #joint1 in base frame -> moves only in base Y axis
joint2_joint1, _, _, _ = track_data.homg_mat_mult(joint1_inv,joint2) #joint2 in joint1 frame -> moves only in J1 Z axis

#J1 motors are 4 and 5
j1_setpoint = np.array([0, -5, 0]) * np.pi/180	#positive values move down, negative up, degrees, is around y
j1_setpoint = track_data.euler2homg_mat(j1_setpoint,'sxyz')	#euler to homgenous matrix
_, _, joint1_setpoint_eul, _ = track_data.homg_mat_mult(joint1_base, j1_setpoint) #converts rotation in joint frame to rotation in base frame
joint1_setpoint_eul = np.array(joint1_setpoint_eul) * 180 / np.pi #setpoint is outputted as euler

#J2 motors are 2 and 7
j2_setpoint = np.array([0, 0, -5]) * np.pi/180#positive values move , negative , is around z
j2_setpoint = track_data.euler2homg_mat(j2_setpoint,'sxyz')
_, _, joint2_setpoint_eul, _ = track_data.homg_mat_mult(joint2_joint1, j2_setpoint)
joint2_setpoint_eul = np.array(joint2_setpoint_eul) * 180 / np.pi

j1_error = 0
j1_error_cum = 0
j1_p_gain = 150
j1_i_gain = 0.2

j2_error = 0
j2_error_cum = 0
j2_p_gain = 150
j2_i_gain = 0.2

counter = 0

while True:
	#if NatNet.frame > prev_frame:
	track_data.parse_data(NatNet.joint_data, NatNet.frame) #updates the frame and data that is being used

	base = track_data.bodies[0].homogenous_mat
	base_inv = track_data.bodies[0].homg_inv
	joint1 = track_data.bodies[1].homogenous_mat
	joint1_inv = track_data.bodies[1].homg_inv
	joint2 = track_data.bodies[2].homogenous_mat
	joint2_inv = track_data.bodies[2].homg_inv

	joint1_base, j1b_pos, j1b_euler, j1b_quat = track_data.homg_mat_mult(base_inv,joint1) #joint1 in base frame -> moves only in base Y axis
	joint2_joint1, j2j1_pos, j2j1_euler, j2j1_quat = track_data.homg_mat_mult(joint1_inv,joint2) #joint2 in joint1 frame -> moves only in J1 Z axis

	j1b_deg = np.array(j1b_euler) * 180 / np.pi
	j2j1_deg = np.array(j2j1_euler) * 180 / np.pi

	j1_error = np.sum(joint1_setpoint_eul - j1b_deg)
	j1_error_cum = j1_error_cum + j1_error

	j2_error = np.sum(joint2_setpoint_eul - j2j1_deg)
	j2_error_cum = j2_error_cum + j2_error

	motors.motor_pos[3] = int(start_pos[3] + (j1_error * j1_p_gain) + (j1_error_cum * j1_i_gain))
	motors.motor_pos[4] = int(start_pos[4] - (j1_error * j1_p_gain) - (j1_error_cum * j1_i_gain))

	motors.motor_pos[1] = int(start_pos[1] - (j2_error * j2_p_gain) - (j2_error_cum * j2_i_gain))
	motors.motor_pos[6] = int(start_pos[6] + (j2_error * j2_p_gain) + (j2_error_cum * j2_i_gain))

	#motors.command_motors(motors.motor_pos)

	if counter % 10 == 0:
		print("joint1 current pos euler", j1b_deg)
		print("joint1 setpoint euler", joint1_setpoint_eul)
		print("joint1 error:", j1_error)
		print("joint2 current pos euler", j2j1_deg)
		print("joint2 setpoint euler", joint2_setpoint_eul)
		print("joint2 error:", j2_error)
		print("\n")


	counter = counter + 1
	prev_frame = track_data.frame
	time.sleep(.01) #100 hz