from math import cos, sin

def location(theta1, theta2):
	joint1 = 1 # length of first joint
	joint2 = 2 # length of second joint

	x_1 = joint1 * cos(theta1)
	y_1 = joint1 * cos(theta1)
	# print("x_1: ", x_1, "y_1: ", y_1)
	x_2 = x_1 + joint2 * cos(theta1 + theta2)
	y_2 = y_1 + joint2 * sin(theta1 + theta2)
	# print("x_2: ", x_2, "y_2: ", y_2)