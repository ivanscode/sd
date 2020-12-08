import cv2
import numpy as np
from Math_Functions import *

PIXELS_PER_METER = 120
BORDER_WIDTH = 0

BLACK = (0,0,0)
WHITE = (255,255,255)
GRAY = (122, 122, 122)
ROYGBIV = [
		(255, 0, 0),
		(255, 127, 0),
		(255, 255, 0),
		(0, 255, 0),
		(0, 0, 255),
		(46, 43, 95),
		(139, 0, 255)
]
ROYGBIV = list([x[2], x[1], x[0]] for x in ROYGBIV)

def rainbow_color(index): return ROYGBIV[index % len(ROYGBIV)]

class Drawer():
	def __init__(self):
		self.min_x = 0
		self.min_y = 0
		self.max_x = 0
		self.max_y = 0
		self.window_width = 0
		self.window_height = 0
		self.img = None

	def x_to_pixel(self, x): return int((x - self.min_x) * PIXELS_PER_METER + BORDER_WIDTH)

	def y_to_pixel(self, y): return int(self.window_height - ((y - self.min_y) * PIXELS_PER_METER + BORDER_WIDTH))

	def xy_to_pixel(self, x, y): return self.x_to_pixel(x), self.y_to_pixel(y)

	def draw_wall(self, w):
		cv2.line(self.img, self.xy_to_pixel(w.x0(), w.y0()), self.xy_to_pixel(w.x1(), w.y1()), BLACK, 1)


	def draw_points(self, points, color):
		for p in points:
			cv2.circle(self.img, self.xy_to_pixel(p.x(), p.y()), 3, color, -1)


	def draw_node_segments(self, node, sameColor=False, dedensified=False):
		segments = node.get_pred_segments(dedensified=dedensified)
		for i in range(len(segments)):
			color = rainbow_color(i if not sameColor else 0)
			self.draw_points(segments[i], color)

	def draw(self, nodes, walls, quickclose=False, change_colors=True):
		self.draw_surface(nodes, walls, change_colors)

	def draw_surface(self, nodes, walls, change_colors, dedensified=False):
		self.min_x = min(list(min(p.x() for p in n.get_pred_points()) for n in nodes) + list(w.minx() for w in walls))
		self.min_y = min(list(min(p.y() for p in n.get_pred_points()) for n in nodes) + list(w.miny() for w in walls))

		self.max_x = max(list(max(p.x() for p in n.get_pred_points()) for n in nodes) + list(w.maxx() for w in walls))
		self.max_y = max(list(max(p.y() for p in n.get_pred_points()) for n in nodes) + list(w.maxy() for w in walls))

		self.window_width = int((self.max_x - self.min_x) * PIXELS_PER_METER + BORDER_WIDTH * 2)
		self.window_height = int((self.max_y - self.min_y) * PIXELS_PER_METER + BORDER_WIDTH * 2)

		img = np.zeros((self.window_height, self.window_width, 3), np.uint8)
		img.fill(255)
		self.img = img

		for w in walls:
			self.draw_wall(w)
		for n in nodes:
			self.draw_points([Point(n.pred_x, n.pred_y)], BLACK)
			self.draw_points(n.get_pred_points(dedensified=dedensified), BLACK)
			self.draw_node_segments(n, sameColor=not change_colors, dedensified=dedensified)

def draw_to_image(nodes, walls, fname, change_colors=True):
	d = Drawer()
	d.draw_surface(nodes, walls, change_colors=change_colors), fname
	cv2.imwrite(fname, d.img)