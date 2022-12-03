from kinematics import ForwardKinematics
import numpy as np

# def go_to_location(start_pose, goal):

def estimate_jacobian(kin: ForwardKinematics, pose: np.ndarray):
	kin.getPos(pose)
