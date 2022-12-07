from robot import Robot
from svg import parse_svg
from svg_renderers import PhysicalRenderer
import numpy as np
import time

def terminal():
	arm = Robot()
	arm.print_status()
	instructions = """Enter a command:
	p x y z: move to location x y z in centimeters
	d a1 a2 a3 a4 a5 a6: set joint angles to the specified angles in degrees
	Any angles set to - will keep their current values.
	"""
	while True:
		command = input(instructions)
		if command.startswith('p'):
			loc = np.array([float(x) for x in command.split()[1:]])
			arm.move2location(loc)
		elif command.startswith('d'):
			angles = arm.logical_angles.copy()
			for i, value in enumerate(command.split()[1:]):
				if value != '-':
					angles[i] = int(value) * np.pi / 180
			arm.move2pose(angles)
		else:
			print("Invalid command")
			continue
		time.sleep(3)
		arm.print_status()

def draw():
	svg = parse_svg('svg_files/face.svg')
	arm = Robot()
	renderer = PhysicalRenderer(svg, arm)
	# renderer = MatPlotLibRenderer(svg)
	for shape in svg.shapes:
		print('Rendering shape', shape)
		shape.render_with(renderer)
	# renderer.show()

if __name__ == "__main__":
	draw()