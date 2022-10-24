#!/usr/bin/env python3

from dis import dis
from re import A

from numpy import matrix
from arms import RoboticArm2DoFSim
from math import pi
import math
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B  # type: ignore
import sys
from inverseKin import inverse_kinematics, eval_robot
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

	def draw_line(self, x1, y1, x2, y2):
		"""Draw a line from (x1, y1) to (x2, y2)"""
		delta = Matrix.from_array([[x2 - x1], [y2 - y1]])
		delta.normalize()


	def get_position(self):
		return self.sim.location_with_angles(self.joint1.position * pi / 180, self.joint2.position * pi / 180)

	def print_status(self):
		print("Joint 1: {} Joint 2: {}".format(self.joint1.position, self.joint2.position), file=sys.stderr)
		x, y = self.get_position()
		print("{:.4f}, {:.4f}".format(x, y), file=sys.stderr)
