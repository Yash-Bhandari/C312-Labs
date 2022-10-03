import math

def distance(x0, y0, x1, y1):
	return ((x1-x0)**2 + (y1-y0)**2)**0.5

def angle_of_intersecting_lines(p0, p1, p2):
	"""Computes the angle of the lines p0p2 and p0p2"""
	# we use law of cosines here
	x0, y0 = p0
	x1, y1 = p1
	x2, y2 = p2
	# C is the angle between the two lines, c is the length of the line connecting the point 1 and point 2
	a = distance(x0, y0, x1, y1) # length of line 1 (point 0 to point 1)
	b = distance(x0, y0, x2, y2) # length of line 2 (point 0 to point 2)
	c = distance(x1, y1, x2, y2) # length of line 3 (point 1 to point 2)
	C = math.acos((c**2 - a**2 - b**2) / (-2 * a * b))
	return C