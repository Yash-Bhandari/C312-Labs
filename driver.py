#!/usr/bin/env python3

from arms import RoboticArm2DoFSim
from math import pi
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B  # type: ignore
from time import sleep
from ev3dev2.button import Button
import sys

class RoboticArm:
	def __init__(self, j1, j2, joint1_port = OUTPUT_A, joint2_port= OUTPUT_B) -> None:
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

	def get_position(self):
		return self.sim.location_with_angles(self.joint1.position * pi / 180, self.joint2.position * pi / 180)

	def print_status(self):
		print("Joint 1: {} Joint 2: {}".format(self.joint1.position, self.joint2.position), file=sys.stderr)
		x, y = self.get_position()
		print("{:.4f}, {:.4f}".format(x, y), file=sys.stderr)


if __name__ == "__main__":
	j1_length = 0.117 
	arm = RoboticArm(0.117, 0.096)
	button = Button()
	sleep(1)

	def go_to_angle(a1, a2):
		arm.set_angles(a1, a2)
		print(arm.get_position(), file=sys.stderr)
		button.wait_for_bump('enter')
	go_to_angle(90, 90)
	go_to_angle(135, -90)
	go_to_angle(0, 0)

	# test moving
	# arm.print_status()
	# while True:
	# 	button.wait_for_bump('enter')
	# 	arm.print_status()