from robot import Robot
from time import *
from kinematics import ForwardKinematics, InverseKinematics
import numpy as np


arm = Robot()


pose = [180, 116.4, 90, 95, 160, 90] #116.4
# pose = [1.571, 1.571, 1.571, 1.571, 2.793, 1.571]

pose = [np.radians(pose[i]) for i in range(6)]

arm.move2pose(pose)

origin = [-8.35, 0, 2.6] 

# cheack [4.05, 0, 6.25]

# joints = [[4.05, 0, 6.25], [0, 0, 9], [-0.85, 0, 2.165],
#           [15.8, 0, 0], [1.825, 0, 0.85], [0, 0, 0]]

joints = [[4.05, 0, 6.75], [9, 0, 0], [-2.165, 0, 0.85],
          [15.8, 0, 0], [1.825, 0, 0.85], [0, 0, 0]]


kin = ForwardKinematics(joints, origin)

logical = kin.physicalToLogicalAngles(pose)
Output = ["%.2f" % np.degrees(elem) for elem in logical]
print(Output)

# test_pose = kin.logicalToPhysicalAngles(pose_new)

pos = kin.getPos(logical)
print(pos[0:3])

user = input("stop: ")

arm.stopRobot()
