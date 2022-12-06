from robot import Robot
import numpy as np
import time

def terminal():
	arm = Robot()
	arm.print_status()
	instructions = """Enter a command:
	p x y z: move to location x y z in centimeters
	d a1 a2 a3 a4 a5 a6: set joint angles to the specified angles in degrees
	"""
	while True:
		command = input(instructions)
		if command.startswith('p'):
			loc = np.array([float(x) for x in command.split()[1:]])
			arm.move2location(loc)
		elif command.startswith('d'):
			angles = np.array([int(x) for x in command.split()[1:]]) / 180 * np.pi
			arm.move2pose(angles)
		else:
			print("Invalid command")
			continue
		time.sleep(3)
		arm.print_status()

if __name__ == "__main__":
	terminal()