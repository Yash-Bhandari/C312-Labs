#!/usr/bin/env python3

from dis import dis
from re import A
from arms import RoboticArm2DoFSim
from math import pi
import math
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B  # type: ignore
from time import sleep
from ev3dev2.button import Button
from utils import distance, angle_of_intersecting_lines
import sys
from inverseKin import inverse_kinematics
from matrix import Matrix


JOINT1_LENGTH = 0.113
JOINT2_LENGTH = 0.095

class RoboticArm:
	def __init__(self, j1 = JOINT1_LENGTH, j2 = JOINT2_LENGTH, joint1_port = OUTPUT_A, joint2_port= OUTPUT_B) -> None:
		"""
		Args: 
			j1 (float): length of first joint
			j2 (float): length of second joint
		"""
		self.speed = 20
		self.sim = RoboticArm2DoFSim(j1, j2)
		self.joint1 = LargeMotor(joint1_port)
		self.joint2 = LargeMotor(joint2_port)
		self.joint1.reset()
		self.joint2.reset()

	def set_angles(self, theta1, theta2):
		"""Set the angles of the joints in degrees"""
		self.joint1.on_to_position(5, theta1)
		self.joint2.on_to_position(5, theta2)

		theta1 = theta1 * 180 / pi
		theta2 = theta2 * 180 / pi
		pred_x, pred_y = self.sim.location_with_angles(theta1, theta2)
		print("Predicted Location: {:.3f}, {:.3f}".format(pred_x, pred_y))

	def go_to_position(self, x, y, method = 'analytical'):
		"""Move the arm to the given location"""
		if method == 'analytical':
			theta1, theta2 = self.sim.angles_for_location(x, y)
		elif method == 'numerical':

			START_THETA = Matrix(2,1)
			START_THETA[0][0] = self.joint1.position()
			START_THETA[1][0] = self.joint2.position()
			l = Matrix(2,1)
			l[0][0] = JOINT1_LENGTH
			l[1][0] = JOINT2_LENGTH
			pos = Matrix(2,1)
			pos[0][0] = x
			pos[1][0] = y
			theta1, theta2 = inverse_kinematics(l, START_THETA, pos, 20, 'newton')
		self.set_angles(theta1 * 180 / pi, theta2 * 180 / pi)

	def get_position(self):
		return self.sim.location_with_angles(self.joint1.position * pi / 180, self.joint2.position * pi / 180)

	def print_status(self):
		print("Joint 1: {} Joint 2: {}".format(self.joint1.position, self.joint2.position), file=sys.stderr)
		x, y = self.get_position()
		print("{:.4f}, {:.4f}".format(x, y), file=sys.stderr)

# q2 part a
def repeated_angle_test():
	arm = RoboticArm()
	button = Button()

	def go_to_angle(a1, a2):
		arm.set_angles(a1, a2)
		print(arm.get_position(), file=sys.stderr)
		button.wait_for_bump('enter')

	theta1 = 30
	theta2 = 50
	for trial in range(5):
		print('Trial {}'.format(trial), file=sys.stderr)
		go_to_angle(theta1, theta2)

# q2 c1
def measure_distance():
	arm = RoboticArm()
	button = Button()

	for trial in range(5):
		print('Trial {}'.format(trial), file=sys.stderr)
		button.wait_for_bump('enter')
		x0, y0 = arm.get_position()
		arm.print_status()
		button.wait_for_bump('enter')
		x1, y1 = arm.get_position()
		arm.print_status()
		distance = ((x1-x0)**2 + (y1-y0)**2)**0.5
		print('Distance: {:.2f}cm'.format(distance*100), file=sys.stderr)

# q2 cii
def measure_angle():
	arm = RoboticArm()
	button = Button()

	button.wait_for_bump('enter')
	p0 = arm.get_position() # point 0, intersection of the two lines
	arm.print_status()
	button.wait_for_bump('enter')
	p1 = arm.get_position() # point 1, first line
	arm.print_status()
	button.wait_for_bump('enter')
	p2 = arm.get_position() # point 2, second line
	arm.print_status()
	angle = angle_of_intersecting_lines(p0, p1, p2) * 180 / pi
	print("Angle between lines: {:}".format(angle), file=sys.stderr)


# q3 ai
def go_to_position():
	arm = RoboticArm()
	button = Button()
	positions = [(0.15, 0.1), (0.15, -0.1), (0, 0.15)]
	for x, y in positions:
		print('Trying to go to ({:.2f}, {:.2f})'.format(x, y), file=sys.stderr)
		arm.go_to_position(x, y)
		arm.print_status()
		button.wait_for_bump('enter')

# q3 aii
def midpoint():
	arm = RoboticArm()
	button = Button()

	# Let user specify two points
	button.wait_for_bump('enter')
	x0, y0 = arm.get_position() 
	arm.print_status()
	button.wait_for_bump('enter')
	x1, y1 = arm.get_position() # point 1, first line
	arm.print_status()

	x_mid = (x0 + x1) / 2
	y_mid = (y0 + y1) / 2
	print('Midpoint: ({:.3f}, {:.3f})'.format(x_mid, y_mid), file=sys.stderr)
	arm.go_to_position(x_mid, y_mid)


if __name__ == "__main__":
	repeated_angle_test()
	# measure_angle()
	# midpoint()
	# arm = RoboticArm()
	# button = Button()
	# # test moving
	# arm.print_status()
	# while True:
	# 	button.wait_for_bump('enter')
	# 	arm.print_status()