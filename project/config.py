from dataclasses import dataclass
WORLD = [-8.35, 0, 2.4] 

# == joint measurements (cm) == 
JOINTS = [[4.05, 0, 6], [9, 0, 0], [-0.85, 0, 2.165],
        [15.8, 0, 0], [2, 0, 1], [0.8, 0, -7.5]]

# == cup coordinates (cm) == 
L_CUP_0 = [-3.1, 14.2, -5.7]
L_CUP_1 = [-9.5, 14.2, -5.7]

R_CUP_0 = [-3.1, -14.2, -5.7]
R_CUP_1 = [-9.5, -14.2, -5.7]

# == canvas (cm) == 
CANVAS_ANGLE = 0.4904375 #rads (28.1 deg)
CANVAS_BL = [25, 14.5, 11]
CANVAS_BR = [25, -14.5, ]
CANVAS_FL = [6, 14.5, 0]
CANVAS_FR = [6, -14.5, 0]

STARTING_POSE_PHYSICAL = [1.570, 2.443, 2.094, 1.570, 2.792, 3.141] # ture acuator angles used to drive the model 


@dataclass
class CanvasDims:
	"""
	Represents the dimensions of the canvas
	"""
	width: float # cm
	height: float # cm
	x_offset: float # cm
	y_offset: float # cm
	z_offset: float # cm
	slant: float # degrees


closest_peg = CanvasDims(15, 15, 9, -10, 3, 0.565) # 32 degrees

# CANVAS = CanvasDims(15, 15, 9, -10, 3, 0.565) # 32 degrees

CANVAS = closest_peg