from svg import Shape, RectSVG, CircleSVG, EllipseSVG, LineSVG, PolyLineSVG, PolygonSVG, PathSVG, SVG, SVGRenderer
import numpy as np
from math import pi
import matplotlib.pyplot as plt

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
		xs = np.linspace(shape.x1, shape.x2, self.num_points)
		ys = np.linspace(shape.y1, shape.y2, self.num_points)
		return np.stack((xs, ys), axis=1)

	def render_circle(self, shape: CircleSVG):
		theta = np.linspace(0, 2 * pi, self.num_points)
		xs = np.cos(theta) * shape.radius + shape.center_x
		ys = np.sin(theta) * shape.radius + shape.center_y
		return np.stack((xs, ys), axis=1)

class MatPlotLibRenderer(SVGRenderer):
	"""
	This renderer draws a image using matplotlib
	"""

	def __init__(self, svg: SVG):
		super().__init__(svg)
		self.np_renderer = NumpyRenderer(svg)

	def render_line(self, line: LineSVG):
		points = self.np_renderer.render_line(line)
		# draw the line with 
		plt.plot(points[:, 0], points[:, 1])
		plt.show()

	def render_circle(self, circle: CircleSVG):
		points = self.np_renderer.render_circle(circle)
		# plot should always be the same size:
		dimension = int(max(self.svg.width, self.svg.height))
		plt.xlim([0, dimension])
		plt.ylim([0, dimension])
		plt.axis()
		plt.plot(points[:, 0], points[:, 1], )
		# axis ranges should be the same
		plt.gca().set_aspect('equal', adjustable='box')
		plt.show()
		