from svg import PathCommand

def test_path_command():
	command = PathCommand('m 1 2,3 4')
	assert command.type == PathCommand.Type.Move
	assert command.relative
	assert command.values == [1, 2, 3, 4]