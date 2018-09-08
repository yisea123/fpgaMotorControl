from forwardKinematics import robot_config
import numpy as np
pi = np.pi
q = [-pi/2, pi/2, -pi/2, 20]
#q = [0, 0, 0, 20]

myRobot = robot_config()

# calculate position of the end-effector
xyz = myRobot.Tx('j4', q)
print('XYZ forward kin: {}'.format(xyz))
# calculate the Jacobian for the end effector
#JEE = myRobot.J('j4', q)

