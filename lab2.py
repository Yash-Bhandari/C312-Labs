#!/usr/bin/env python3
from driver import RoboticArm
import sys
from ev3dev2.button import Button
from time import sleep
from utils import distance, angle_of_intersecting_lines


# q2 part a
def locations():
	arm = RoboticArm()
	button = Button()
	while True:
		arm.print_status()
		button.wait_for_bump('enter')

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
	locations()
	# measure_angle()
	# go_to_position()
	# midpoint()

	# arm = RoboticArm()
	# button = Button()
	# # test moving
	# arm.print_status()
	# while True:
	# 	button.wait_for_bump('enter')
	# 	arm.print_status()