import pygame
from Math_Functions import Point
import Wall

PIXELS_PER_METER = 1.5
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

BUILD_SIZE = 30, 30

def rainbow_color(index): return ROYGBIV[index % len(ROYGBIV)]

class Drawer():
	def __init__(self):
		self.min_x = 0
		self.min_y = 0
		self.max_x = 0
		self.max_y = 0
		self.window_width = 0
		self.window_height = 0

	def x_to_pixel(self, x): return int((x - self.min_x) * PIXELS_PER_METER + BORDER_WIDTH)

	def y_to_pixel(self, y): return int(self.window_height - ((y - self.min_y) * PIXELS_PER_METER + BORDER_WIDTH))

	def xy_to_pixel(self, x, y): return self.x_to_pixel(x), self.y_to_pixel(y)

	def draw_wall(self, w, surface):
		pygame.draw.line(surface, BLACK, self.xy_to_pixel(w.x0(), w.y0()), self.xy_to_pixel(w.x1(), w.y1()), 1)


	def draw_points(self, points, color, surface):
		for p in points:
			pygame.draw.circle(surface, color, self.xy_to_pixel(p.x(), p.y()), 3)


	def draw_node_segments(self, node, surface, sameColor=False):
		segments = node.get_pred_segments()
		for i in range(len(segments)):
			color = rainbow_color(i if not sameColor else 0)
			self.draw_points(segments[i], color, surface)

	def mousexy(self):
		x, y = pygame.mouse.get_pos()

		return (x - BORDER_WIDTH) / PIXELS_PER_METER + self.min_x, \
			   (-(y - self.window_height) - BORDER_WIDTH) / PIXELS_PER_METER + self.min_y

	def draw(self, nodes, walls, quickclose=False, change_colors=True):
		self.draw_surface(nodes, walls, change_colors)
		pygame.display.update()
		while True:
			for event in pygame.event.get():
				if event.type == pygame.QUIT:
					break
				if event.type == pygame.MOUSEBUTTONDOWN:
					if quickclose: break
					print('\tPoint(%f,\t%f),' % self.mousexy())
			else:
				continue
			break

	def draw_surface(self, nodes, walls, change_colors):
		pygame.init()
		self.min_x = min(list(min(p.x() for p in n.get_pred_points()) for n in nodes) + list(w.minx() for w in walls))
		self.min_y = min(list(min(p.y() for p in n.get_pred_points()) for n in nodes) + list(w.miny() for w in walls))

		self.max_x = max(list(max(p.x() for p in n.get_pred_points()) for n in nodes) + list(w.minx() for w in walls))
		self.max_y = max(list(max(p.y() for p in n.get_pred_points()) for n in nodes) + list(w.miny() for w in walls))

		self.window_width = (self.max_x - self.min_x) * PIXELS_PER_METER + BORDER_WIDTH * 2
		self.window_height = (self.max_y - self.min_y) * PIXELS_PER_METER + BORDER_WIDTH * 2

		gameDisplay = pygame.display.set_mode((int(self.window_width), int(self.window_height)))
		gameDisplay.fill(WHITE)
		for w in walls:
			self.draw_wall(w, gameDisplay)
		for n in nodes:
			self.draw_points([Point(n.real_x, n.real_y)], BLACK, gameDisplay)
			self.draw_points(n.get_pred_points(), BLACK, gameDisplay)
			self.draw_node_segments(n, gameDisplay,sameColor=not change_colors)
		return gameDisplay

	def build(self):
		pygame.init()
		self.min_x = - BUILD_SIZE[0] / 2
		self.min_y = - BUILD_SIZE[1] / 2

		self.max_x = BUILD_SIZE[0] / 2
		self.max_y = BUILD_SIZE[1] / 2

		self.window_width = (self.max_x - self.min_x) * PIXELS_PER_METER + BORDER_WIDTH * 2
		self.window_height = (self.max_y - self.min_y) * PIXELS_PER_METER + BORDER_WIDTH * 2

		gameDisplay = pygame.display.set_mode((int(self.window_width), int(self.window_height)))

		last_pt = None
		walls = []
		while True:
			for event in pygame.event.get():
				if event.type == pygame.QUIT:
					break
				if event.type == pygame.MOUSEBUTTONDOWN:
					print('\tPoint(%f,\t%f),' % self.mousexy())
					new_pt = Point(*self.mousexy())
					if last_pt:
						walls.append(Wall.Wall(last_pt, new_pt))
					last_pt = new_pt

					gameDisplay.fill(WHITE)
					for w in walls:
						self.draw_wall(w, gameDisplay)
			else:
				continue
			break
		return gameDisplay

def draw(nodes, walls, quickclose=False, change_colors=True):
	Drawer().draw(nodes, walls, quickclose=quickclose, change_colors=change_colors)

def draw_to_image(nodes, walls, fname, change_colors=True):
	pygame.image.save(Drawer().draw_surface(nodes, walls, change_colors=change_colors), fname)


if __name__ == '__main__':
	Drawer().build()