#!/usr/bin/env python3

from dis import dis
from re import A

from arms import RoboticArm2DoFSim
from math import pi
from time import sleep
from utils import distance
import math
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B  # type: ignore
from ev3dev2.button import Button
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
		self.j1_length = j1
		self.j2_length = j2
		self.joint1 = LargeMotor(joint1_port)
		self.joint2 = LargeMotor(joint2_port)
		self.reset()

	def reset(self):
		self.joint1.reset()
		self.joint2.reset()

	def set_angles(self, theta1, theta2, brake=True, block=True):
		"""Set the angles of the joints in degrees"""
		if theta2 > 180:
			theta2 = theta2 - 360
		self.joint1.on_to_position(5, theta1, brake=brake, block=False)
		self.joint2.on_to_position(5, theta2, brake=brake, block=block)

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
		print("Trying to go to start position: {:.4f}, {:.4f}".format(x1, y1), file=sys.stderr)
		self.go_to_position(x1, y1, method='analytical')
		delta = Matrix.from_array([[x2 - x1], [y2 - y1]])
		delta.normalize()

		angles = Matrix.from_array([[self.joint1.position], [self.joint2.position]])

		button = Button()
		print('Press the button to start', file=sys.stderr)
		button.wait_for_bump('enter')
		step_size = 5 # step size in degrees
		sleep_time = 0.2 # time to sleep between steps
		while distance(x2, y2, *self.get_position()) > 0.02:
			dist = distance(x2 - x1, y2 - y1, *self.get_position())
			if dist < 0.1 and step_size > 1:
				step_size = step_size / 2
			J = self.sim.jacobian(self.joint1.position * pi / 180, self.joint2.position * pi / 180)
			if J.det() == 0:
				J = self.sim.jacobian(self.joint1.position * pi / 180 + 0.1, self.joint2.position * pi / 180 + 0.1)
			angles = Matrix.from_array([[self.joint1.position], [self.joint2.position]])
			theta_dot = J.inverse() * delta 
			angles = angles + theta_dot.scale(180 / pi).normalize().scale(step_size)
			theta1, theta2 = angles[0][0] , angles[1][0] 
			self.set_angles(theta1 , theta2, brake=True, block=False)
			sleep(sleep_time)
			print('Setting joints to theta1: {}, theta2: {}'.format(theta1 , theta2 ), file=sys.stderr)
			# button.wait_for_bump('enter')
		# self.set_angles(angles[0][0] , angles[1][0], brake=True, block=True)
	def get_position(self):
		"""Returns the current estimated position of the end effector"""
		return self.sim.location_with_angles(self.joint1.position * pi / 180, self.joint2.position * pi / 180)

	def print_status(self):
		print("Joint 1: {} Joint 2: {}".format(self.joint1.position, self.joint2.position), file=sys.stderr)
		x, y = self.get_position()
		print("{:.4f}, {:.4f}".format(x, y), file=sys.stderr)

if __name__ == "__main__":
	button = Button()
	arm = RoboticArm()
	arm.draw_line(0.15, 0, 0, 0.15)
	# print('Press enter to reset the arm', file=sys.stderr)
	# button.wait_for_bump('enter')
	# arm.print_status()
	# x0, y0 = arm.get_position()
	# button.wait_for_bump('enter')
	# arm.print_status()
	# x1, y1 = arm.get_position()
	# arm.draw_line(x1, y1, x0, y0)