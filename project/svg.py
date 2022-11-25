import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Literal, Dict, List, Tuple
from enum import Enum
import typing
import sys
import re


@dataclass
class Shape:
	type: Literal['rect', 'circle', 'ellipse', 'line', 'polyline', 'polygon', 'path']
	params: Dict

@dataclass
class RectSVG(Shape):
	x: int # top left corner
	y: int
	width: int
	height: int

@dataclass
class CircleSVG(Shape):
	center_x: int
	center_y: int
	radius: int

@dataclass
class EllipseSVG(Shape):
	center_x: int
	center_y: int
	radius_x: int
	radius_y: int

@dataclass
class LineSVG(Shape):
	x1: int
	y1: int
	x2: int
	y2: int

@dataclass
class PolyLineSVG(Shape):
	points: List[Tuple[int]]

@dataclass 
class PolygonSVG(Shape):
	points: List[Tuple[int]]

@dataclass
class PathSVG(Shape):
	pass

	def __init__(self, params):
		r'([a-zA-Z][^a-zA-Z]+)'

		self.description = description
		self.commands = []

@dataclass
class PathCommand:
	class Type(str, Enum):
		Move = 'M'
		Line = 'L'
		Horizontal = 'H'
		Vertical = 'V'
		ClosePath = 'Z' # return to first point in path

	type: Type
	relative: bool
	values: List[int]

	def __init__(self, command: str):
		# first character is the command type
		self.type = PathCommand.Type(command[0].upper())
		self.relative = command[0].islower()
		# split on spaces and commas
		breakpoint()
		values = map(int, re.split(r'[\s,]+', command[1:]))
		

@dataclass
class SVG:
	width: int
	height: int
	shapes: List[Shape] = field(default_factory=list)

	def __str__(self):
		return f"SVG(width={self.width}, height={self.height})"

	def __repr__(self):
		return f"SVG(width={self.width}, height={self.height})"


def parse_svg(path: str):
	tree = ET.parse(path)
	root = tree.getroot()
	width = int(root.attrib["width"])
	height = int(root.attrib["height"])
	svg = SVG(width, height)
	for child in root:
		shape = parse_shape(child)
		svg.shapes.append(shape)
	return svg

def parse_shape(element: ET.Element) -> Shape:
	# '{http://www.w3.org/2000/svg}rect' -> 'rect'
	shape_type = element.tag.split('}')[1]
	shape = Shape(shape_type, element.attrib)
	return shape

if __name__ == "__main__":
	path = sys.argv[1] if len(sys.argv) > 1 else "example.svg"
	svg = parse_svg(path)