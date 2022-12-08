from svg import parse_svg
from svg_renderers import MatPlotLibRenderer
svg = parse_svg('svg_files/face.svg')
# renderer = PhysicalRenderer(svg, arm)
renderer = MatPlotLibRenderer(svg)
for shape in svg.shapes:
	print('Rendering shape', shape)
	shape.render_with(renderer)
renderer.show()
