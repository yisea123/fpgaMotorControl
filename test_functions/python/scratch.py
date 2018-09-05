import numpy as np
import time

#Motor arm mixing
Db = 16.5
Ds = 8.72
Dm = 13.88
Dl = 18
Dj1 = 21
Dj2 = 21.5
Dj3 = 13.5

motorTheta_armTheta_old = np.zeros((4,4))
motorTheta_armTheta_old[0,0] = Db/Dj1
motorTheta_armTheta_old[1,0] = Dl/Dj2 * motorTheta_armTheta_old[0,0]
motorTheta_armTheta_old[1,1] = Db/Dj2
motorTheta_armTheta_old[2,0] = Dm/Dj3 * motorTheta_armTheta_old[0,0]
motorTheta_armTheta_old[2,1] = -Dm/Dj3 * motorTheta_armTheta_old[1,1]
motorTheta_armTheta_old[2,2] = Db/Dj3
motorTheta_armTheta_old[3,0] = Ds * motorTheta_armTheta_old[0,0]
motorTheta_armTheta_old[3,1] = -Ds * motorTheta_armTheta_old[1,1]
motorTheta_armTheta_old[3,2] = Dl * motorTheta_armTheta_old[2,2]
motorTheta_armTheta_old[3,3] = Db

motorTheta_armTheta = np.eye(4) * np.array([Db/Dj1, Db/Dj2, Db/Dj3, Db])
motorTheta_armTheta = np.dot(np.array([[1, 0, 0, 0], [Dl/Dj2, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]), motorTheta_armTheta)
motorTheta_armTheta = np.dot(np.array([[1, 0, 0, 0], [0, 1, 0, 0], [Dm/Dj3, -Dm/Dj3, 1, 0], [0, 0, 0, 1]]), motorTheta_armTheta)
motorTheta_armTheta = np.dot(np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [Ds, -Ds, Dl, 1]]), motorTheta_armTheta)

armTheta_motorTheta = np.linalg.inv(motorTheta_armTheta)

print(motorTheta_armTheta_old)
print(motorTheta_armTheta)
print(armTheta_motorTheta)