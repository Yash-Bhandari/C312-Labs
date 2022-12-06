from robot import Robot
import numpy as np

def terminal():
	arm = Robot()
	while True:
		command = input("Enter command as x y z: ")
		loc = np.array([float(x) for x in command.split()])
		arm.move2location(loc)
		arm.print_status()

if __name__ == "__main__":
	terminal()