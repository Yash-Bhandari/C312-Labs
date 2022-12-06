from robot import Robot
import numpy as np
import time

def terminal():
	arm = Robot()
	arm.print_status()
	while True:
		command = input("Enter command as x y z: ")
		loc = np.array([float(x) for x in command.split()])
		arm.move2location(loc)
		time.sleep(3)
		arm.print_status()

if __name__ == "__main__":
	terminal()