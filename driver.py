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
		self.reset()

	def reset(self):
		self.joint1.reset()
		self.joint2.reset()

	def set_angles(self, theta1, theta2):
		"""Set the angles of the joints in degrees"""
		if theta2 > 180:
			theta2 = theta2 - 360
		self.joint1.on_to_position(5, theta1, brake=False)
		self.joint2.on_to_position(5, theta2, brake=False)

		theta1 = theta1 / 180 * pi
		theta2 = theta2 / 180 * pi
		pred_x, pred_y = self.sim.location_with_angles(theta1, theta2)
		print("Predicted Location: {:.3f}, {:.3f}".format(pred_x, pred_y), file=sys.stderr)

	def go_to_position(self, x, y, method = 'analytical'):
		"""Move the arm to the given location"""
		if method == 'analytical':
			theta1, theta2 = self.sim.angles_for_location(x, y)
			self.set_angles(theta1 * 180 / pi, theta2 * 180 / pi)
			print('Setting joints to theta1: {}, theta2: {}'.format(theta1 * 180 / pi, theta2 * 180 /pi ), file=sys.stderr)
		elif method == 'numerical':
			start_x, start_y = self.get_position()
			delta_x = x - start_x
			delta_y = y - start_y
			# Split the journey into 3 steps
			for step in range(1, 4):
				# prepare matrices to pass into the inverse_kinematics methods
				start_angles = Matrix(2,1)
				start_angles[0][0] = self.joint1.position * pi / 180
				start_angles[1][0] = self.joint2.position * pi / 180
				l = Matrix(2,1)
				# joint length matrix
				l[0][0] = JOINT1_LENGTH
				l[1][0] = JOINT2_LENGTH
				# position matrix, travelling 1/3 of the way each time
				pos = Matrix(2,1)
				pos[0][0] = start_x + delta_x * step / 3
				pos[1][0] = start_y + delta_y * step / 3
				theta1, theta2 = inverse_kinematics(l, start_angles, pos, 20, 'newton')
				# convert to degrees
				theta1 = (theta1 * 180 / pi ) % 360
				theta2 = (theta2 * 180 / pi ) % 360
				self.set_angles(theta1, theta2)
			print('Setting joints to theta1: {}, theta2: {}'.format(theta1, theta2), file=sys.stderr)

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

	theta1 = 90
	theta2 = -90
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

	while True:
		print('Press enter to start', file=sys.stderr)
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
	positions = [(0.05, 0.15), (0.1, 0.1)]
	arm.print_status()
	print('Press the button to start!', file=sys.stderr)
	button.wait_for_bump('enter')
	for x, y in positions:
		print('Trying to go to ({:.2f}, {:.2f})'.format(x, y), file=sys.stderr)
		arm.go_to_position(x, y, method='numerical')
		arm.print_status()
		button.wait_for_bump('enter')
		arm.reset()

# q3 aii
def midpoint():
	arm = RoboticArm()
	button = Button()
	while True:
		print('Press the button to start!', file=sys.stderr)
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
		arm.go_to_position(x_mid, y_mid, method='numerical')


if __name__ == "__main__":
	# repeated_angle_test()
	# measure_distance()
	# measure_angle()
	go_to_position()
	# midpoint()

	# arm = RoboticArm()
	# button = Button()
	# # test moving
	# arm.print_status()
	# while True:
	# 	button.wait_for_bump('enter')
	# 	arm.print_status()
