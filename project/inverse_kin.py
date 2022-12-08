from kinematics import ForwardKinematics
import numpy as np
import random

def generate_random_valid_angles(kin):
	angles = np.random.uniform(0, np.pi, 6)
	kin.jointLimitsPhysical(angles).T
	return np.array(angles)

def pose_for_location(kin: ForwardKinematics, start_pose, goal, threshold=0.1, retry_count = 7):
	max_movement = 0.6 # the total movement of all arms must be less than .6 rads per step
	pose = kin.logicalToPhysicalAngles(start_pose)
	for attempt in range(retry_count):
		bounds = kin.jointLimitsPhysical(pose).T
		for iter in range(100):
			position = kin.getPos(pose, physical=True)
			delta_y = goal - position
			distance = np.linalg.norm(delta_y)
			if distance < threshold:
				return kin.physicalToLogicalAngles(pose)

			jacobian = estimate_jacobian(kin, pose)
			# solving for delta_x in J * delta_x = delta_y
			try:
				delta_x = np.linalg.pinv(jacobian) @ delta_y
			except np.linalg.LinAlgError:
				breakpoint()
				jacobian += np.random.uniform(-0.01, 0.01, 6)
			# make sure we don't move past the joint limits
			for i in range(len(delta_x)):
				if pose[i] + delta_x[i] < bounds[0,i] or pose[i] + delta_x[i] > bounds[1,i]:
					delta_x[i] = bounds[0,i] - pose[i]
			magnitude = np.linalg.norm(delta_x)
			direction = delta_x / magnitude
			delta_x = direction * min(magnitude, max_movement)
			for i in range(len(delta_x)):
				if pose[i] + delta_x[i] < bounds[0,i] or pose[i] + delta_x[i] > bounds[1,i]:
					delta_x[i] = bounds[0,i] - pose[i]
			pose += delta_x
		if attempt == 0:
			pose = np.array([1.570, 2.443, 2.094, 1.71042, 1.8, 1.48353])
			print(f"Warning: IK did not converge. Trying again from start pose")
		if attempt != retry_count - 1:
			pose = generate_random_valid_angles(kin)
			print(f"Warning: IK did not converge. Randomizing starting pose")
		else:
			raise Exception(f"inverse kinematics failed. goal: {goal} closest: {position}") 
	# assert kin.physicalToLogicalAngles(kin.logicalToPhysicalAngles(start_pose)) == start_pose
	return kin.physicalToLogicalAngles(pose)
		
def estimate_jacobian(kin: ForwardKinematics, pose: np.ndarray):
	y = kin.getPos(pose, physical=True)
	if np.isnan(y := kin.getPos(pose, physical=True)).any():
		pose += np.random.uniform(-0.01, 0.01, 6)
	jacobian = np.zeros((3, 6), np.float64)
	h = 0.0001
	for i in range(len(pose)):
		pose[i] += h
		y_new = kin.getPos(pose, physical=True)
		pose[i] -= h
		derivative = (y_new - y) / h
		if np.isnan(derivative).any():
			jacobian[:,i] = 0
		else:
			jacobian[:,i] = derivative
	return jacobian