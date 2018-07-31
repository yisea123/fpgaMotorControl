
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