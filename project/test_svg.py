from svg import PathCommand, PathSVG, parse_svg

def test_path_command():
	command = PathCommand('m 1 2,3 4')
	assert command.type == PathCommand.Type.Move
	assert command.relative
	assert command.values == [1, 2, 3, 4]

def test_path_svg():
	d = "M20,230 Q40,205 50,230 T90,230"
	path = PathSVG({'d': d})
	assert len(path.commands) == 3
	assert path.commands[0].type == PathCommand.Type.Move
	assert path.commands[0].values == [20, 230]
	assert path.commands[1].type == PathCommand.Type.QuadraticBezier
	assert path.commands[1].values == [40, 205, 50, 230]
	assert path.commands[2].type == PathCommand.Type.StrungQuadraticBezier
	assert path.commands[2].values == [90, 230]


def test_can_parse_viewbox():
	svg = parse_svg('svg_files/face.svg')
	assert svg.width == 497.92
	assert svg.height == 583.13