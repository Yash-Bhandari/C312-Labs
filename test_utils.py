import utils
from math import pi, isclose
import pytest

@pytest.mark.parametrize("p0, p1, p2, true_answer", [
	[(0, 0), (1, 0), (0, 1), pi / 2],
	[(0, 0), (-1, -1), (1, -1), pi / 2],
	[(0, 0), (-1, -1), (1, 0), 3 * pi / 4],
	[(0, 0), (1, 1), (-1, 1), pi / 2],
	[(0, 1), (1, 0), (0.5, 2), 1.8925],
])
def test_angle_of_intersecting_lines(p0, p1, p2, true_answer):
	angle = utils.angle_of_intersecting_lines(p0, p1, p2)
	assert isclose(angle, true_answer, abs_tol = 0.001)