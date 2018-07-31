from NatNetClient2 import NatNetClient
import numpy as np
import time
import signal



class data():
	def __init__(self, names, ids, timestamp = 0):
		self.bodies = []
		for name, id in zip(names, ids):
			self.bodies.append(self.body(name,id))
		self.timestamp = timestamp


	#Updates the position, orientation and timestamp of the values
	def parse_data(self,joint_data):
		self.timestamp = joint_data[1]
		for i in range(len(joint_data[0])):
			self.bodies[joint_data[0][i][0]].position = joint_data[0][i][1]
			self.bodies[joint_data[0][i][0]].orientation = joint_data[0][i][2]
		return

	class body():
		def __init__(self, name, id):
			self.name = name
			self.id = id
			self.position = None
			self.orientation = None


def signal_handler(signal, frame):
	streamingClient.stop()
	sys.exit(0)

def receiveNewFrame( frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
	labeledMarkerCount, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged):

	global frame
	frame = frameNumber
	#print( "Received frame", frameNumber)
	#print("Number of rigid bodies", rigidBodyCount)
	#print("Timestamp", timestamp)


def receiveRigidBodyFrame( id, position, rotation ): #this function is looped in the system call with an offset to change the id
	#print("id: {}, position: {}, rotation: {}" .format(id,position,rotation))
	pass

def receiveRigidBodyFrameList(rigid_body_list, timestamp): #receives all the information at once
	global joint_data
	joint_data = [rigid_body_list, timestamp]

signal.signal(signal.SIGINT, signal_handler)

server_ip = "192.168.1.4"
multicastAddress = "239.255.42.99"
print_data = False

streamingClient = NatNetClient(server_ip, multicastAddress, verbose = print_data)

# Configure the streaming client to call our rigid body handler on the emulator to send data out.
streamingClient.newFrameListener = receiveNewFrame
streamingClient.rigidBodyListener = receiveRigidBodyFrame
streamingClient.rigidBodyListListener = receiveRigidBodyFrameList

#Global values from function calls
frame = 0
prev_frame = 0
joint_data = 0

print("Starting streaming client now...")

time.sleep(1)
streamingClient.run()
time.sleep(1)

joint_names = ['base', 'joint1', 'joint2']
ids = [0,1,2]
data = data(joint_names,ids)
# bodies = []
# for name, id in zip(joint_names, ids):
# 	bodies.append(body(name,id))
# data = data(bodies,0)

while True:
	if frame > prev_frame:
		data.parse_data(joint_data)
		print("joint data is:", joint_data)
		for i in range(len(joint_data[0])):
			print(data.bodies[i].name)
			print(data.bodies[i].id)
			print(data.bodies[i].position)
			print(data.bodies[i].orientation)

		print("\n")
		prev_frame = frame