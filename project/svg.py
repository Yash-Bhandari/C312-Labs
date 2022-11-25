import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Literal, Dict, List, Tuple
from enum import Enum
import typing
import sys
import re


@dataclass
class Shape:
	# type: Literal['rect', 'circle', 'ellipse', 'line', 'polyline', 'polygon', 'path']
	params: Dict = field(kw_only=True, repr=False)
	stroke: str 
	stroke_width: int 
	fill: str 

	def __init__(self, params: Dict):
		self.params = params
		self.stroke = params.get('stroke', 'black')
		self.stroke_width = params.get('stroke-width', 1)
		self.fill = params.get('fill', 'transparent')

@dataclass
class RectSVG(Shape):
	x: float # top left corner
	y: float
	width: float
	height: float

	def __init__(self, params: Dict):
		super().__init__(params)
		self.x = float(params['x'])
		self.y = float(params['y'])
		self.width = float(params['width'])
		self.height = float(params['height'])

@dataclass
class CircleSVG(Shape):
	center_x: float
	center_y: float
	radius: float

	def __init__(self, params: Dict):
		super().__init__(params)
		self.center_x = float(params['cx'])
		self.center_y = float(params['cy'])
		self.radius = float(params['r'])

@dataclass
class EllipseSVG(Shape):
	center_x: float
	center_y: float
	radius_x: float
	radius_y: float

	def __init__(self, params: Dict):
		super().__init__(params)
		self.center_x = float(params['cx'])
		self.center_y = float(params['cy'])
		self.radius_x = float(params['rx'])
		self.radius_y = float(params['ry'])

@dataclass
class LineSVG(Shape):
	x1: float
	y1: float
	x2: float
	y2: float

	def __init__(self, params: Dict):
		super().__init__(params)
		self.x1 = float(params['x1'])
		self.y1 = float(params['y1'])
		self.x2 = float(params['x2'])
		self.y2 = float(params['y2'])

@dataclass
class PolyLineSVG(Shape):
	points: List[Tuple[float, float]]

	def __init__(self, params: Dict):
		super().__init__(params)
		self.points = []
		values = [float(x) for x in params['points'].split()]
		if len(values) % 2 != 0:
			raise ValueError('Invalid number of values in polyline points')
		for i in range(0, len(values), 2):
			self.points.append((values[i], values[i + 1]))

@dataclass 
class PolygonSVG(Shape):
	points: List[Tuple[float, float]]

	def __init__(self, params: Dict):
		super().__init__(params)
		self.points = []
		values = [float(x) for x in params['points'].split()]
		if len(values) % 2 != 0:
			raise ValueError('Invalid number of values in polygon points')
		for i in range(0, len(values), 2):
			self.points.append((values[i], values[i + 1]))

@dataclass
class PathCommand:
	class Type(str, Enum):
		Move = 'M'
		Line = 'L'
		Horizontal = 'H'
		Vertical = 'V'
		ClosePath = 'Z' # return to first point in path
		CubicBezier = 'C'
		StrungCubicBezier = 'S' # unsupported
		QuadraticBezier = 'Q'
		StrungQuadraticBezier = 'T' # unsupported

		def __repr__(self):
			return str(self).split('.')[-1]

	type: Type
	relative: bool
	values: List[float]

	def __init__(self, command: str):
		# first character is the command type
		self.type = PathCommand.Type(command[0].upper())
		self.relative = command[0].islower()
		value_strings = re.findall(r'\-?\d*\.?\d*', command[1:])
		# split on spaces and commas
		self.values = [float(x) for x in value_strings if x != '']

@dataclass
class PathSVG(Shape):
	commands: List['PathCommand'] = field(default_factory=list)

	def __init__(self, params):
		super().__init__(params)
		description = params['d']
		command_strings = re.findall(r'([a-zA-Z][^a-zA-Z]+)', description)
		self.commands = [PathCommand(x) for x in command_strings]

	def __repr__(self):
		base = super().__repr__()
		indent = '\n' + ' ' * 4
		return base + indent + indent.join(map(str, self.commands))

@dataclass
class SVG:
	width: int
	height: int
	shapes: List[Shape] = field(default_factory=list)

	def __repr__(self):
		indent = '\n' + ' ' * 2
		shapes = indent + indent.join(map(str, self.shapes))
		return f'SVG(width={self.width}, height={self.height}, shapes={shapes})'

shape_classes = {
	'rect': RectSVG,
	'circle': CircleSVG,
	'ellipse': EllipseSVG,
	'line': LineSVG,
	'polyline': PolyLineSVG,
	'polygon': PolygonSVG,
	'path': PathSVG,
}

def parse_shape(element: ET.Element) -> Shape:
	# '{http://www.w3.org/2000/svg}rect' -> 'rect'
	shape_type = element.tag.split('}')[1]
	shape = shape_classes[shape_type](element.attrib)
	return shape

def parse_svg(path: str):
	tree = ET.parse(path)
	root = tree.getroot()
	width = int(root.attrib.get("width", '200'))
	height = int(root.attrib.get("height", '200'))
	svg = SVG(width, height)
	for child in root:
		shape = parse_shape(child)
		svg.shapes.append(shape)
	return svg

if __name__ == "__main__":
	path = sys.argv[1] if len(sys.argv) > 1 else "svg_files/example.svg"
	svg = parse_svg(path)