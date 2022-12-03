from robot import Robot
from time import *
from kinematics import ForwardKinematics, InverseKinematics
import numpy as np


arm = Robot()


pose = [90, 140, 120, 90, 160, 180]


# pose2 = [np.radians(pose[i]) for i in range(6)]


# arm.move2pose(pose2)


joints = [[4,0,4.8],[0,0,9],[-0.5,0,2.04],
          [16.68,0,0],[1.3,0,0.6],[1.6,0,6.8]]



start = time()

kin = ForwardKinematics()

pos = kin.getPos([1.5707963267948966, 2.443460952792061, 2.0943951023931953, 1.5707963267948966, 2.792526803190927, 3.141592653589793], joints)

end = time()
print("time: ", end - start)

print(pos)





# arm.stopRobot()