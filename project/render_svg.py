from robot import Robot
from svg import parse_svg
from svg_renderers import PhysicalRenderer, MatPlotLibRenderer
import numpy as np
import time

svg = parse_svg('svg_files/face.svg')
renderer = MatPlotLibRenderer(svg)
for shape in svg.shapes:
	shape.render_with(renderer)
renderer.show()

renderer = MatPlotLibRenderer(svg, in_3d=True)
for shape in svg.shapes:
	shape.render_with(renderer)
renderer.show()