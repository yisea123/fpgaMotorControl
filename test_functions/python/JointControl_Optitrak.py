import numpy as np
import time
import signal
import os

from utils.GetJointData import data, receiveNewFrame, receiveRigidBodyFrameList
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
streamingClient = NatNetClient(server_ip, multicastAddress, verbose = print_data)
streamingClient.newFrameListener = receiveNewFrame
streamingClient.rigidBodyListListener = receiveRigidBodyFrameList

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
motors.command_motors(motors.motor_pos)			#echo read value, values arent used since c side is in err mode

#---------------------------------------#---------------------------------------

print("Arming motors now...")
motors.arm()
time.sleep(1)
