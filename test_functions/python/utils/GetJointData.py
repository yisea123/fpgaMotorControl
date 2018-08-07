import transforms3d as tf3d
import numpy as np

class data():
	def __init__(self, names, ids, timestamp = 0):
		#bodies is a list of body class objects
		self.bodies = []
		for name, id in zip(names, ids):
			self.bodies.append(self.body(name,id))
		self.timestamp = timestamp
		self.joint_data = None
		self.frame = None


	#Updates the position, orientation and timestamp of the values
	#parse data updates all the class objets in bodies, all values are from camera to body reference frame
	#----------------------
	#IMPORTANT -> Natnet streams the orientation as a quaternion (qx, qy qz, qw), tf3d library expects (qw, qx, qy, qz)
	#			  orientation order when converting to Euler is x (pitch), y (yaw), z (roll), right handed, using relative or local coordinate frame (Motive Software uses local)
	#----------------------
	def parse_data(self,joint_data, frame):
		self.joint_data = joint_data
		self.frame = frame
		self.timestamp = joint_data[1]
		#Loops over the number of rigid bodies in the scene
		for i in range(len(joint_data[0])):
			self.bodies[joint_data[0][i][0]].position = joint_data[0][i][1]
			self.bodies[joint_data[0][i][0]].orientation = np.array(joint_data[0][i][2])[[3,0,1,2]] #reorders oreintation to (qw, qx, qy, qz)
			self.bodies[joint_data[0][i][0]].rotation_mat = tf3d.quaternions.quat2mat(self.bodies[joint_data[0][i][0]].orientation)

			euler = tf3d.euler.quat2euler(self.bodies[joint_data[0][i][0]].orientation, 'rxyz')
			self.bodies[joint_data[0][i][0]].euler = euler

			hm_mat = np.vstack([np.hstack([self.bodies[joint_data[0][i][0]].rotation_mat,np.array(joint_data[0][i][1])[:,None]]),np.array([0,0,0,1])[None,:]])
			self.bodies[joint_data[0][i][0]].homogenous_mat = hm_mat
			self.bodies[joint_data[0][i][0]].homg_inv = self.homg_inv(hm_mat)
		return

	#inverts homogeneous matrix
	def homg_inv(self, hm_mat):
		rotation = hm_mat[0:3,0:3] #3 by 3
		translation = hm_mat[0:3,3][:,None] #3 by 1
		translation_inv = -np.matmul(rotation.T,translation) #3 by 1
		hm_mat_inv = np.vstack([np.hstack([rotation.T, translation_inv]),np.array([0,0,0,1])[None,:]])
		return hm_mat_inv

	#transforms coordinate frame from hm_mat2 to hm_mat1 (inv matrix)
	def homg_mat_mult(self, hm_mat1, hm_mat2):
		hm_mat = np.matmul(hm_mat1,hm_mat2)
		hm_mat_position = hm_mat[0:3,3]
		hm_mat_euler = tf3d.euler.mat2euler(hm_mat[0:3,0:3], 'sxyz') # x (pitch), y (yaw), z (roll), using static here different than relative as in the motive software
		hm_mat_quat = tf3d.quaternions.mat2quat(hm_mat[0:3,0:3])
		return hm_mat, hm_mat_position, hm_mat_euler, hm_mat_quat

	def euler2homg_mat(self, euler_angles, axes):
		rot_mat = tf3d.euler.euler2mat(euler_angles[0], euler_angles[1], euler_angles[2], axes)
		homg_mat = np.vstack([np.hstack([rot_mat, np.zeros(3)[:,None]]),np.array([0,0,0,1])[None,:]])
		return homg_mat


	#look at https://github.com/NxRLab/ModernRobotics/blob/master/code/Python/modern_robotics.py


	class body():
		def __init__(self, name, id):
			self.name = name
			self.id = id
			self.position = None
			self.orientation = None #quaternion
			self.euler = None #mat2euler
			self.rotation_mat = None
			self.homogenous_mat = None
			self.homg_inv =  None

"""
Need to use globals for passing back values from another thread, if using a large number of threads then a solutiong
is to use Queues -> I think this would be very very large number of trheads, out of scope of this work
"""
class NatNetFuncs():
	def __init__(self):
		self.frame = 0
		self.joint_data = 0

	def receiveNewFrame(self, frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
		labeledMarkerCount, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged):
		#global frame
		self.frame = frameNumber
		#print( "Received frame", frameNumber)
		#print("Number of rigid bodies", rigidBodyCount)
		#print("Timestamp", timestamp)


	def receiveRigidBodyFrame(self, id, position, rotation ): #this function is looped in the system call with an offset to change the id
		#print("id: {}, position: {}, rotation: {}" .format(id,position,rotation))
		pass

	def receiveRigidBodyFrameList(self, rigid_body_list, timestamp): #receives all the information at once
		#global joint_data
		self.joint_data = [rigid_body_list, timestamp]