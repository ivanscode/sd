from Math_Functions import *
import LS2D
from Wall import Wall
import math

# Measurements greater will be thrown out. For real data only.
THROW_OUT_THRESHOLD = 3000

# All simulation parameters below
# LIDAR
L_RANGE = 12 # meters
L_DISTANCE_TOLERANCE = .1 # meters
L_MEASUREMENTS = 800

# Node-Node
N_DISTANCE_TOLERANCE = .1

# For display use only
WALL_DISTANCE_SCALE = .5

class Node:
	def __init__(self, x, y, angle, number):
		self.real_x = x
		self.real_y = y
		self.real_angle = angle

		self.number = number

		self.pred_x = 0
		self.pred_y = 0
		self.pred_angle = 0

		self._wall_measurements = [] # Pts
		self._segments = [] # Pts separated into walls
		self._relative_walls = [] # Now line segments

		self._node_measurements = []

	def set_measurement_file(self, path):
		with open(path) as f:
			lines = f.readlines()

		for i in range(len(lines)):
			if int(lines[i]) > THROW_OUT_THRESHOLD: continue
			self._wall_measurements.append(pt_thetar(math.pi * 2 / len(lines) * i, float(lines[i])))

	# ======================================================================================================================================
	# Getters for translated values using this node's predicted locations
	# ======================================================================================================================================
	def get_pred_points(self):
		return list(p.translate(self.pred_x, self.pred_y, self.pred_angle) for p in self._wall_measurements)

	def get_pred_segments(self):
		return list(list(p.translate(self.pred_x, self.pred_y, self.pred_angle) for p in s) for s in self._segments)

	def get_pred_walls(self):
		return list(w.translate(self.pred_x, self.pred_y, self.pred_angle) for w in self._relative_walls)

	def get_pred_wall(self, index):
		return self._relative_walls[index].translate(self.pred_x, self.pred_y, self.pred_angle)

	# ======================================================================================================================================
	# For simulation & generating mock data
	# ======================================================================================================================================
	def sim_scan(self, nodes, walls):
		print('Node %d beginning scanning' % self.number)
		for n in nodes:
			self._node_measurements.append((tolerance(math.hypot(self.real_x-n.real_x, self.real_y-n.real_y), N_DISTANCE_TOLERANCE), n))

		print('Scanning for walls.')
		for step in range(0, L_MEASUREMENTS):
			angle = step * math.pi * 2 / L_MEASUREMENTS
			distance = self.sim_lidar_measurement(angle + self.real_angle, walls, L_RANGE)
			if distance:
				print('\tAngle %.4f, distance %.2f' % (angle, distance))
				#print(distance)
				self._wall_measurements.append(pt_thetar(angle, distance))


	def sim_lidar_measurement(self, angle, walls, range):
		measured = None
		for w in walls:
			distance = distance_to(self.real_x, self.real_y, angle, w.x0(), w.y0(), w.x1(), w.y1())
			if distance: measured = min(measured, distance) if measured else distance

		if not measured or measured > range:
			return None

		return tolerance(measured, L_DISTANCE_TOLERANCE)
	# ======================================================================================================================================
	# For SLAM
	# ======================================================================================================================================
	def get_relative_walls(self): return self._relative_walls

	def wall_count(self): return len(self._relative_walls)

	def segment(self):
		self._segments = LS2D.LS2D(self._wall_measurements)
		self._relative_walls = []
		for s in self._segments:
			self._relative_walls.append(Wall(*odr_line(s)))

	def get_distance_to_node(self, node):
		for n in self._node_measurements:
			if n[1] is node: return n[0]
		return None

	def get_angle_to_node(self, node):
		return angle_to(self.pred_x, self.pred_y, node.pred_x, node.pred_y)

	def sync_rotations(self, other, myindex, otherindex, orientation, debug_output=False):
		mywall = self.get_relative_walls()[myindex]
		otherwall = other.get_relative_walls()[otherindex]

		try:
			r = math.acos((mywall.perpr() - otherwall.perpr()) / self.get_distance_to_node(other))
		except ValueError:
			return None

		if orientation:
			self.pred_angle = restrict_to_unit_circle(self.get_angle_to_node(other) + r - mywall.perpt())
			other.pred_angle = restrict_to_unit_circle(self.get_angle_to_node(other) + r - otherwall.perpt())
		else:
			self.pred_angle = restrict_to_unit_circle(self.get_angle_to_node(other) - r - mywall.perpt())
			other.pred_angle = restrict_to_unit_circle(self.get_angle_to_node(other) - r - otherwall.perpt())

		if debug_output:
			print('Syncing rotations.')
			print('\tMy wall: ' + str(mywall))
			print('\tOther wall: ' + str(otherwall))
			print('\tr: %f' % r)
			print('\tSet my pred angle to %f' % self.pred_angle)
			print('\tSet other pred angle to %f' % other.pred_angle)

		return self.get_pred_wall(myindex), other.get_pred_wall(otherindex)