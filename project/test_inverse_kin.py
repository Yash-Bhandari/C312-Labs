from kinematics import ForwardKinematics
from inverse_kin import pose_for_location
import numpy as np
import pytest
import math

# -- robot parameters --
JOINTS = [[4.05, 0, 6.75], [0, 0, 9], [-0.85, 0, 2.165],
			  [15.8, 0, 0], [1.825, 0, 0.85], [1.55, 0, -6.75]]
ORIGIN = [-8.35, 0, 2.6] 


@pytest.mark.parametrize('goal', [
	([10, 5, 10]),
	# ([15, 8, 1]),
	([10, 8, 1]),
	([0, 1, 0]),
	([0, 5, 10]),
	([3, 10, 0]),
	([7, 1, 2]),
	([4, -5, 5]),
	([2, -5, 1]),
	([5, -10, 3]),
	([2, -5, 10]),
	([-3, 14.5, -5])
	])
def test_pose_for_location(goal):
	kin = ForwardKinematics(JOINTS, ORIGIN)
	start_pose = np.array([1.57, 2.443, 2.094, 1.57, 2.792, 3.141], np.float64)
	# start_pose = np.random.uniform(0, math.pi, 6)
	goal = np.array(goal)
	pose = pose_for_location(kin, start_pose, goal)
	estimated_position = kin.getPos(pose)
	assert np.allclose(estimated_position, goal, atol=0.1)
	bounds = kin.jointLimitsLogical()
	assert (pose >= bounds[0,:]).all()
	assert (pose <= bounds[1,:]).all()

def test_is_valid():
	kin = ForwardKinematics(JOINTS, ORIGIN)
	angles = [0, 180, 180, 0, 0, 0]
	physical = np.array(np.radians(angles))
	assert not kin.is_valid(physical)
