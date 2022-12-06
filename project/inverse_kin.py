from kinematics import ForwardKinematics
import numpy as np
import random

def generate_random_valid_angles(bounds):
	angles = np.random.uniform(bounds[0], bounds[1], 6)
	angles = []
	for i in range(6):
		angles.append(random.uniform(bounds[0,i], bounds[1,i]))
	return np.array(angles)
def pose_for_location(kin: ForwardKinematics, start_pose, goal, threshold=0.01, retry_count = 7):
	max_movement = 0.6 # the total movement of all arms must be less than .6 rads per step
	pose = start_pose
	bounds = kin.jointLimitsLogical()
	for attempt in range(retry_count):
		for iter in range(100):
			position = kin.getPos(pose)
			delta_y = goal - position
			distance = np.linalg.norm(delta_y)
			if distance < threshold:
				return pose

			jacobian = estimate_jacobian(kin, pose)
			# solving for delta_x in J * delta_x = delta_y
			delta_x = np.linalg.pinv(jacobian) @ delta_y
			# make sure we don't move past the joint limits
			for i in range(len(delta_x)):
				if pose[i] + delta_x[i] < bounds[0,i]:
					delta_x[i] = bounds[0,i] - pose[i]
				elif pose[i] + delta_x[i] > bounds[1,i]:
					delta_x[i] = bounds[1,i] - pose[i]

			magnitude = np.linalg.norm(delta_x)
			direction = delta_x / magnitude
			movement = direction * min(magnitude, max_movement)
			pose += movement
		pose = generate_random_valid_angles(bounds)
		print(f"Warning: IK did not converge. Randomizing starting pose")
	return pose
		
def estimate_jacobian(kin: ForwardKinematics, pose: np.ndarray):
	y = kin.getPos(pose)
	jacobian = np.zeros((3, 6), np.float64)
	h = 0.0001
	for i in range(len(pose)):
		pose[i] += h
		y_new = kin.getPos(pose)
		pose[i] -= h
		derivative = (y_new - y) / h
		jacobian[:,i] = derivative
	return jacobian