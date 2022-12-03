from kinematics import ForwardKinematics
import numpy as np

def go_to_location(kin: ForwardKinematics, start_pose, goal, threshold=0.1):
	# max_movement = 0.6 # the total movement of all arms must be less than .6 rads per step
	pose = start_pose
	while True:
		position = kin.getPos(pose)
		delta_y = goal - position
		distance = np.linalg.norm(delta_y)
		if distance < threshold:
			break

		jacobian = estimate_jacobian(kin, pose)
		# solving for delta_x in J * delta_x = delta_y
		delta_x = np.linalg.pinv(jacobian) * delta_y
		# magnitude = np.linalg.norm(delta_x)
		# direction = delta_x / magnitude
		# movement = direction * min(magnitude, max_movement)
		pose += delta_x
	return pose

def estimate_jacobian(kin: ForwardKinematics, pose: np.ndarray):
	y = kin.getPos(pose)
	jacobian = np.zeros((3, 6))
	h = 0.0001
	for i in range(len(pose)):
		pose[i] += h
		y_new = kin.getPos(pose)
		pose[i] -= h
		derivative = (y_new - y) / h
		jacobian[:,i] = derivative
	return jacobian
