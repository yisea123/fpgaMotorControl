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


while 1:
	track_data.parse_data(NatNet.joint_data, NatNet.frame) #updates the frame and data that is being used
	print(track_data.frame)

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


	print("joint 2 position in base frame\n base: {0[0]:.4f}, {0[1]:.4f}, {0[2]:.4f}".format(j2b_pos_mm))
	print("joint 2 euler in base frame\n base: {0[0]:.4f}, {0[1]:.4f}, {0[2]:.4f}\n".format(j2b_deg))
	print("joint 3 position in joint 2 frame\n base: {0[0]:.4f}, {0[1]:.4f}, {0[2]:.4f}".format(j3j2_pos_mm))
	print("joint 3 euler in joint 2 frame\n base: {0[0]:.4f}, {0[1]:.4f}, {0[2]:.4f}\n".format(j3j2_deg))
	print("joint 4 position in joint 3 frame\n base: {0[0]:.4f}, {0[1]:.4f}, {0[2]:.4f}".format(j4j3_pos_mm))
	print("joint 4 euler in joint 3 frame\n base: {0[0]:.4f}, {0[1]:.4f}, {0[2]:.4f}\n".format(j4j3_deg))
	time.sleep(0.1)