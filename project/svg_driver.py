from svg import parse_svg
import config 
from svg_renderers import NumpyRenderer, ImageTo3D
from inverse_kin import pose_for_location
from kinematics import ForwardKinematics
import time

arm = None
svg = parse_svg('svg_files/face.svg')
renderer = NumpyRenderer(svg)
converter = ImageTo3D(config.CANVAS, svg)
kin = ForwardKinematics()
pose = kin.physicalToLogicalAngles(config.STARTING_POSE_PHYSICAL)
for shape in svg.shapes:
	print('Rendering shape', shape)
	points = shape.render_with(renderer)
	for point in points:
		coords = converter.image_to_3d(point)
		pose = pose_for_location(kin,pose,coords)
		print(f'Goal: {coords} -> {pose}')
		# time.sleep()

