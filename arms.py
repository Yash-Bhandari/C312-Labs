from math import cos, sin, pi, acos, asin, atan2

class RoboticArm:
	def __init__(self, j1, j2) -> None:
		"""
		Initialize the arm with the lengths of the joints
		Args:
			j1 (float): length of first joint
			j2 (float): length of second joint
		"""
		self.j1 = j1
		self.j2 = j2

	def location_with_angles(self, theta1, theta2):
		"""What location is reached with the given angles"""
		x_1 = self.j1 * cos(theta1)
		y_1 = self.j1 * sin(theta1)
		print("x_1: {:.3f}, y_1: {:.3f}".format(x_1, y_1))
		x_2 = x_1 + self.j2 * cos(theta1 + theta2)
		y_2 = y_1 + self.j2 * sin(theta1 + theta2)
		print("x_2: {:.3f}, y_2: {:.3f}".format(x_2, y_2))
		return x_2, y_2

	def angles_for_location(self, x, y):
		"""What angles are needed to reach the given location"""
		theta2 = acos(x**2 + y**2 - self.j1**2 - self.j2**2) / (2 * self.j1 * self.j2)
		theta1 = asin((self.j1 *sin(theta2)) / (x**2 + y**2)**0.5) - atan2(y, x)
		print("theta1: {:.3f}, theta2: {:.3f}".format(theta1, theta2))
		return theta1, theta2

if __name__ == "__main__":
	arm = RoboticArm(1, 1)
	arm.location_with_angles(0, pi)
	arm.angles_for_location(1, 0.5)