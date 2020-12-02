import Node
import Wall
import pygame
from Math_Functions import Point
import SLAM
import Drawer
import random

PIXELS_PER_METER = .7
BORDER_WIDTH = 100
PILLAR_RADIUS = .4

# The walls are drawn by a turtle walked one by one through these points
WALL_POINTS = [
	# x,	y
	# Square
	Point(-5,	-5),
	Point(-5,	5),
	Point(5,	5),
	Point(5,	-5),

	# Wacky room
	#Point(-2.066667, 7.233333),
	#Point(-8.566667, 0.166667),
	#Point(-3.933333, -6.733333),
	#Point(4.066667, -7.733333),
	#Point(3.966667, -4.166667),
	#Point(6.133333, -1.766667),
	#Point(5.066667, 2.666667),
	#Point(7.933333, 6.733333),
	#Point(1.533333, 9.166667),


	# Long boi
	#Point(-5,	-5),
	#Point(-5,	5),
	#Point(5,	5),
	#Point(5,	-5),

	# Last level
	#Point(-17.750000, -14.600000),
	#Point(-13.300000, -18.200000),
	#Point(-7.750000, -13.300000),
	#Point(-6.650000, -0.600000),
	#Point(-1.850000, -2.200000),
	#Point(1.400000, -7.150000),
	#Point(-1.250000, -12.250000),
	#Point(0.850000, -17.900000),
	#Point(13.100000, -9.700000),
	#Point(6.050000, -3.200000),
	#Point(18.350000, 15.100000),
	#Point(12.950000, 16.850000),
	#Point(1.150000, 3.500000),
	#Point(-2.450000, 8.500000),
	#Point(0.950000, 12.400000),
	#Point(-6.350000, 14.450000),
	#Point(-9.200000, 9.500000),
	#Point(-7.700000, 5.650000),
	#Point(-10.900000, 1.600000),
	#Point(-14.300000, 4.450000),
	#Point(-17.000000, -2.300000),
	#Point(-14.450000, -8.000000),
	#Point(-18.250000, -12.100000),
]

NODE_LOCATIONS = [
	# x,	y,		angle
	(0,		4,		1.5),
	#(0,		-4, 	1.5)

	# Last level
	#(-14.500000, -13.900000),
	#(-11.300000, -3.000000),
	#(-3.550000, 2.850000),
	#(5.200000, 3.300000),
	#(13.550000, 12.800000),
	#(3.050000, -5.050000),
	#(3.050000, -10.050000),
]

PILLARS = [
	Point(-12.307876, -7.645206),
	Point(-12.307876, -7.845206),
	Point(2.392124, 0.004794),
	Point(-14.407876, -0.795206),
	Point(8.692124, -10.095206),
	Point(9.242124, 7.654794),
	Point(-7.407876, 1.354794),
	Point(-9.357876, -11.095206),
]

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

FILES = [
	'ivan_room.txt',
]
def rainbow_color(index):
	return ROYGBIV[index % len(ROYGBIV)]


def x_to_pixel(x):
	return int((x - min(p.x() for p in WALL_POINTS)) * PIXELS_PER_METER + BORDER_WIDTH)


def y_to_pixel(y):
	ROOM_WIDTH = max(p.x() for p in WALL_POINTS) - min(p.x() for p in WALL_POINTS)
	ROOM_HEIGHT = max(p.y() for p in WALL_POINTS) - min(p.y() for p in WALL_POINTS)
	WINDOW_WIDTH = ROOM_WIDTH * PIXELS_PER_METER + BORDER_WIDTH * 2
	WINDOW_HEIGHT = ROOM_HEIGHT * PIXELS_PER_METER + BORDER_WIDTH * 2

	return int(WINDOW_HEIGHT - ((y - min(p.y() for p in WALL_POINTS)) * PIXELS_PER_METER + BORDER_WIDTH))


def mousexy():
	x, y = pygame.mouse.get_pos()
	ROOM_HEIGHT = max(p.y() for p in WALL_POINTS) - min(p.y() for p in WALL_POINTS)
	WINDOW_HEIGHT = ROOM_HEIGHT * PIXELS_PER_METER + BORDER_WIDTH * 2
	return (x - BORDER_WIDTH) / PIXELS_PER_METER + min(p.x() for p in WALL_POINTS), \
			(-(y - WINDOW_HEIGHT) - BORDER_WIDTH) / PIXELS_PER_METER + min(p.y() for p in WALL_POINTS)


def xy_to_pixel(x, y):
	return x_to_pixel(x), y_to_pixel(y)


def draw_wall(w, surface):
	pygame.draw.line(surface, BLACK, xy_to_pixel(w.x0(), w.y0()), xy_to_pixel(w.x1(), w.y1()), 1)


def draw_points(points, color, surface):
	for p in points:
		pygame.draw.circle(surface, color, xy_to_pixel(p.x(), p.y()), 3)


def draw_node_segments(node, surface, sameColor=False):
	segments = node.get_pred_segments()
	for i in range(len(segments)):
		color = rainbow_color(i if not sameColor else 0)
		draw_points(segments[i], color, surface)


def init_room():
	nodes, walls = [], []
	for i in range(len(NODE_LOCATIONS)):
		n = NODE_LOCATIONS[i]
		nodes.append(Node.Node(n[0], n[1], random.random() * 6.28, i))
	for w in range(len(WALL_POINTS)):
		start = WALL_POINTS[w]
		end = WALL_POINTS[(w + 1) % len(WALL_POINTS)]
		walls.append(Wall.Wall(start, end))

	for p in PILLARS:
		walls.append(Wall.Wall(p.translate(-PILLAR_RADIUS,PILLAR_RADIUS,0),p.translate(PILLAR_RADIUS,PILLAR_RADIUS,0)))
		walls.append(Wall.Wall(p.translate(PILLAR_RADIUS,PILLAR_RADIUS,0),p.translate(PILLAR_RADIUS,-PILLAR_RADIUS,0)))
		walls.append(Wall.Wall(p.translate(PILLAR_RADIUS,-PILLAR_RADIUS,0),p.translate(-PILLAR_RADIUS,-PILLAR_RADIUS,0)))
		walls.append(Wall.Wall(p.translate(-PILLAR_RADIUS,-PILLAR_RADIUS,0),p.translate(-PILLAR_RADIUS,PILLAR_RADIUS,0)))

	for node in nodes:
		node.pred_x = node.real_x
		node.pred_y = node.real_y
		node.sim_scan(nodes, walls)

	for node in nodes:
		node.segment()

	return nodes, walls

def draw(nodes, walls, sameColor=False):
	pygame.init()
	ROOM_WIDTH = max(p.x() for p in WALL_POINTS) - min(p.x() for p in WALL_POINTS)
	ROOM_HEIGHT = max(p.y() for p in WALL_POINTS) - min(p.y() for p in WALL_POINTS)
	WINDOW_WIDTH = ROOM_WIDTH * PIXELS_PER_METER + BORDER_WIDTH * 2
	WINDOW_HEIGHT = ROOM_HEIGHT * PIXELS_PER_METER + BORDER_WIDTH * 2
	gameDisplay = pygame.display.set_mode((int(WINDOW_WIDTH), int(WINDOW_HEIGHT)))
	gameDisplay.fill(WHITE)
	for w in walls:
		draw_wall(w, gameDisplay)

	for n in nodes:
		draw_points([Point(n.real_x, n.real_y)], BLACK, gameDisplay)
		draw_points(n.get_pred_points(), BLACK, gameDisplay)
		draw_node_segments(n, gameDisplay, sameColor=sameColor)
	pygame.display.update()

	while True:
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				break
			if event.type == pygame.MOUSEBUTTONDOWN:
				print('\tPoint(%f,\t%f),' % mousexy())
		else:
			continue
		break


def demo(nodes, walls):
	pygame.init()
	gameDisplay = pygame.display.set_mode((int(WINDOW_WIDTH), int(WINDOW_HEIGHT)))
	gameDisplay.fill(WHITE)
	for w in walls:
		draw_wall(w, gameDisplay)
	pygame.display.update()

	while True:
		for event in pygame.event.get():
			if event.type == pygame.MOUSEBUTTONDOWN: break
		else:
			continue
		break

	for n in nodes:
		draw_points([(n.real_x, n.real_y)], BLACK, gameDisplay)
		draw_points(n.get_pred_points(), BLACK, gameDisplay)
		draw_node_segments(n, gameDisplay, True)
	pygame.display.update()
	gameDisplay.fill(WHITE)
	for w in walls:
		draw_wall(w, gameDisplay)

	while True:
		for event in pygame.event.get():
			if event.type == pygame.MOUSEBUTTONDOWN: break
		else:
			continue
		break

	for n in nodes:
		draw_points([(n.real_x, n.real_y)], BLACK, gameDisplay)
		draw_points(n.get_pred_points(), BLACK, gameDisplay)
		draw_node_segments(n, gameDisplay)
	pygame.display.update()

	while True:
		for event in pygame.event.get():
			if event.type == pygame.MOUSEBUTTONDOWN: break
		else:
			continue
		break

	for n in nodes:
		for s in n.get_pred_walls():
			draw_wall(Wall.Wall(s[0][0], s[0][1], s[1][0], s[1][1]), gameDisplay)
	pygame.display.update()

	while True:
		for event in pygame.event.get():
			if event.type == pygame.QUIT: break
		else:
			continue
		break

def from_files():
	nodes = []
	for i in range(len(FILES)):
		n = Node.Node(0,0,0, i)
		n.set_measurement_file(FILES[i])
		n.segment()
		nodes.append(n)
	return nodes

SIM = False
if SIM:
	# ==========================================================================================================================================
	# INIT
	# ==========================================================================================================================================
	# Initialize nodes and walls, scan
	nodes, walls = init_room()

	#rotate = True
	#if rotate:
	#	nodes[0].sync_rotations(nodes[1], 0, 0, 1)
	Drawer.draw(nodes, walls, change_colors=False)
	Drawer.draw(nodes, walls)

	DO_SLAM = True
	if DO_SLAM:
		s = SLAM.SLAM(nodes)
		s.rotate_nodes()
		Drawer.draw(nodes, walls)

	DRAW = False
	if DRAW: draw(nodes, walls)

	DEMO = False
	if DEMO: demo(nodes, walls)
else:
	nodes = from_files()
	while(WALL_POINTS):
		WALL_POINTS.pop(0)
	WALL_POINTS.append(Point(min(min(p.x() for p in n.get_pred_points()) for n in nodes), min(min(p.y() for p in n.get_pred_points()) for n in nodes)))
	WALL_POINTS.append(Point(max(max(p.x() for p in n.get_pred_points()) for n in nodes), max(max(p.y() for p in n.get_pred_points()) for n in nodes)))
	draw(nodes, [], sameColor=True)
	draw(nodes, [])
