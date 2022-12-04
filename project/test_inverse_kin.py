from kinematics import ForwardKinematics
from inverse_kin import pose_for_location
import numpy as np
import pytest

# -- robot parameters --
JOINTS = [[4.05, 0, 6.75], [0, 0, 9], [-0.85, 0, 2.165],
              [15.8, 0, 0], [1.825, 0, 0.85], [1.55, 0, -6.75]]
ORIGIN = [-8.35, 0, 2.6] 


@pytest.mark.parametrize('goal', [
	([10, 5, 10]),
	([15, 8, 1]),
	([0, 0, 0]),
	([0, 5, 10]),
	([3, 10, 0]),
	])
def test_pose_for_location(goal):
	kin = ForwardKinematics(JOINTS, ORIGIN)
	start_pose = np.array([0, 0, 0, 0, 0, 0], np.float64)
	goal = np.array(goal)
	pose = pose_for_location(kin, start_pose, goal)
	estimated_position = kin.getPos(pose)
	assert np.allclose(estimated_position, goal, atol=0.01)
	