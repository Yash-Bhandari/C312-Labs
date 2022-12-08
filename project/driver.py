from robot import Robot
from svg import parse_svg
from svg_renderers import PhysicalRenderer, MatPlotLibRenderer
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
			arm.move2pose(angles, physical=True)
		else:
			print("Invalid command")
			continue
		arm.print_status()

def draw():
	svg = parse_svg('svg_files/duck.svg')
	arm = Robot()
	renderer = PhysicalRenderer(svg, arm)
	# renderer = MatPlotLibRenderer(svg, in_3d=True)
	for shape in svg.shapes:
		arm.goToStartingPose()
		arm.dipPaint()
		print('Rendering shape', shape)
		shape.render_with(renderer)
	arm.goToStartingPose()
	# renderer.show()

if __name__ == "__main__":
	draw()
