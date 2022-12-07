from svg import Shape, RectSVG, CircleSVG, EllipseSVG, LineSVG, PolyLineSVG, PolygonSVG, PathSVG, SVG, SVGRenderer, PathCommand
from robot import Robot
import numpy as np
from math import pi
from config import CanvasDims, CANVAS
from dataclasses import dataclass
try:
	import matplotlib.pyplot as plt
except ImportError:
	plt = None

class NumpyRenderer(SVGRenderer):
	"""
	This renderer returns a numpy array containing points on the curves defined by each shape.	
	You can change the number of points by setting the num_points attribute.

	Returns:
		A numpy array of shape (num_points, 2) where output[i] is the (x, y) of
		the ith point
	"""
	def __init__(self, svg: SVG, num_points = 50):
		super().__init__(svg)
		self.num_points = num_points # how many points will be returned

	def render_line(self, shape: LineSVG):
		xs = np.linspace(shape.x1, shape.x2, self.num_points // 3)
		ys = np.linspace(shape.y1, shape.y2, self.num_points // 3)
		return np.stack((xs, ys), axis=1)

	def render_circle(self, shape: CircleSVG):
		theta = np.linspace(0, 2 * pi, self.num_points)
		xs = np.cos(theta) * shape.radius + shape.center_x
		ys = np.sin(theta) * shape.radius + shape.center_y
		return np.stack((xs, ys), axis=1)

	def cubic_bezier(self, control_xs, control_ys):
		t = np.linspace(0, 1, 20) # paramter 
		t1 = (1-t)**3
		t2 = 3*t*(1-t)**2
		t3 = 3*t**2*(1-t)
		t4 = t**3
		xs = t1*control_xs[0] + t2*control_xs[1] + t3*control_xs[2]  + t4*control_xs[3]
		ys = t1*control_ys[0] + t2*control_ys[1] + t3*control_ys[2]  + t4*control_ys[3]
		return np.stack((xs, ys), axis=1)

	# def render_ellipse(self, ellipse: EllipseSVG):
	def render_path(self, path: PathSVG):
		points = np.empty((0, 2))
		last_command = None
		last_position = None
		position = np.array([0, 0])
		for command in path.commands:
			if command.type == PathCommand.Type.Move:
				points = np.vstack((points, [command.values]))
			if command.type == PathCommand.Type.CubicBezier:
				control_xs = [position[0]]
				control_ys = [position[1]]
				base_x = position[0] if command.relative else 0
				base_y = position[1] if command.relative else 0
				for i in range(3):
					control_xs.append(command.values[2*i] + base_x)
					control_ys.append(command.values[2*i+1] + base_y)

				new_points = self.cubic_bezier(control_xs, control_ys)
				points = np.vstack((points, new_points))

			if command.type == PathCommand.Type.StrungCubicBezier:
				base_x = position[0] if command.relative else 0
				base_y = position[1] if command.relative else 0
				end_x = base_x + command.values[2]
				end_y = base_y + command.values[3]
				new_points = np.array([[end_x, end_y]])
				points = np.vstack((points, new_points))
				last_control = np.array([last_command.values[-4] + last_position[0], last_command.values[-3] + last_position[1]])
				first_control = position + (position - last_control)
				control_xs = [position[0], first_control[0], base_x + command.values[0], end_x]
				control_ys = [position[1], first_control[1], base_y + command.values[1], end_y]

				new_points = self.cubic_bezier(control_xs, control_ys)
				points = np.vstack((points, new_points))

			last_position = position
			position = points[-1]
			last_command = command
		return points

class MatPlotLibRenderer(SVGRenderer):
	"""
	This renderer draws a image using matplotlib
	"""

	def __init__(self, svg: SVG, **kwargs):
		super().__init__(svg)
		self.np_renderer = NumpyRenderer(svg)
		self.render_args = {
			'marker': '.',
			'linestyle': 'None'
		}

	def render_line(self, line: LineSVG):
		points = self.np_renderer.render_line(line)
		self.render_points(points)

	def render_circle(self, circle: CircleSVG):
		points = self.np_renderer.render_circle(circle)
		# plot should always be the same size:
		dimension = int(max(self.svg.width, self.svg.height))
		self.render_points(points)

	def render_path(self, path: PathSVG):
		points = self.np_renderer.render_path(path)
		self.render_points(points)

	def render_points(self, points: np.ndarray):
		dimension = int(max(self.svg.width, self.svg.height))
		plt.xlim([0, dimension])
		plt.ylim([0, dimension])
		plt.plot(points[:, 0], points[:, 1], **self.render_args)
		plt.gca().set_aspect('equal', adjustable='box')

	def show(self):
		plt.gca().invert_yaxis()
		plt.show()



class ImageTo3D:
	def __init__(self, canvas: CanvasDims, svg: SVG):
		self.canvas = canvas
		self.svg = svg
		self.translation = np.array([self.canvas.x_offset, self.canvas.y_offset, self.canvas.z_offset])
		self.matrix = self.projection_matrix()

	def projection_matrix(self):
		"""
		Input:
			slant of the canvas (rad)
			translation from the origin of the robot to the bottom left corner of the canvas
		output:
			3x3 matrix that maps from the image coordinates to the 3d coordinates
		"""
		matrix = np.zeros((3, 3))
		slant = self.canvas.slant
		matrix[:,2] = np.array([self.canvas.x_offset, self.canvas.y_offset, self.canvas.z_offset])
		matrix[0,1] = 1
		matrix[1,0] = np.cos(slant)
		matrix[2,0] = np.sin(slant)
		# matrix = np.array([
		# 	[0            , 1, x_offset],
		# 	[np.cos(slant), 0, y_offset],
		# 	[np.sin(slant), 0, z_offset],
		# ])
		return matrix

	def image_to_3d(self, point: np.ndarray):
		"""
		Converts a point in the image (pixel coords) to a point in 3d space on
		the canvas.
		"""
		scaled = point.copy()
		scaled[0] = scaled[0] / self.svg.width
		scaled[1] = scaled[1] / self.svg.height
		out = self.matrix[:,:2] @ scaled
		out[0] *= self.canvas.height * np.cos(self.canvas.slant)
		out[1] *= self.canvas.width
		out[2] *= self.canvas.height * np.sin(self.canvas.slant)
		out += self.translation
		return out

class PhysicalRenderer(SVGRenderer):
	"""
	This renderer draws an image on a physical canvas using the robotic arm.
	"""

	def __init__(self, svg: SVG, arm: Robot, canvas: CanvasDims=CANVAS, **kwargs):
		super().__init__(svg)
		self.np_renderer = NumpyRenderer(svg)
		self.arm = arm
		self.converter = ImageTo3D(canvas, svg)

	def render_line(self, line: LineSVG):
		points = self.np_renderer.render_line(line)
		self.render_points(points)

	def render_circle(self, circle: CircleSVG):
		points = self.np_renderer.render_circle(circle)
		self.render_points(points)

	def render_path(self, path: PathSVG):
		points = self.np_renderer.render_path(path)
		self.render_points(points)

	def render_points(self, points: np.ndarray):
		for i in range(points.shape[0]):
			location = self.converter.image_to_3d(points[i])
			print('moving to', location)
			breakpoint()
			self.arm.move2location(location)