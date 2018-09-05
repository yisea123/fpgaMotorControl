import numpy as np

def getOptitrackPose(track_data, NatNet):
	#---------------------------------------#---------------------------------------
	#Get Optitrack data                     #---------------------------------------
	#---------------------------------------#---------------------------------------
	track_data.parse_data(NatNet.joint_data, NatNet.frame) #updates the frame and data that is being used
	#print(track_data.frame)
	
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

	joint4_base, j4b_pos, j4b_euler, _ = track_data.homg_mat_mult(base_inv,joint4)


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

	return j1_angle, j2_angle, j3_angle, j4_pos, joint4_base, j4b_pos, j4b_euler