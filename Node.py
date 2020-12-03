from Math_Functions import *
import LS2D
from Wall import Wall
import math
from Drawer import draw

# Measurements greater will be thrown out. For real data only.
THROW_OUT_THRESHOLD = 12

# All simulation parameters below
# LIDAR
L_RANGE = 12 # meters
L_DISTANCE_TOLERANCE = 5 * LS2D.START_SIGMA # meters
L_MEASUREMENTS = 200

# Node-Node
N_DISTANCE_TOLERANCE = .1

# For display use only
WALL_DISTANCE_SCALE = .5

ROTATE_ROUNDS = 4
ROTATE_SPEED = .01

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
		self._dedensified_measurements = []

	def set_measurement_file(self, path):
		with open(path) as f:
			lines = f.readlines()

		for i in range(len(lines)):
			distance = float(lines[i].strip()) / 100
			if distance > THROW_OUT_THRESHOLD: continue
			self._wall_measurements.append(pt_thetar(math.pi * 2 / len(lines) * i, distance))

	def from_list(self, measurements):
		for i in range(len(measurements)):
			distance = measurements[i] / 100
			if distance > THROW_OUT_THRESHOLD: continue
			self._wall_measurements.append(pt_thetar(math.pi * 2 / len(measurements) * i, distance))

	# ======================================================================================================================================
	# Getters for translated values using this node's predicted locations
	# ======================================================================================================================================
	def get_pred_points(self, index=-1, dedensified=False):
		if index == -1:
			if dedensified:
				return list(p.translate(self.pred_x, self.pred_y, self.pred_angle) for p in self._dedensified_measurements)
			return list(p.translate(self.pred_x, self.pred_y, self.pred_angle) for p in self._wall_measurements)
		else:
			if dedensified:
				return self._dedensified_measurements[index].translate(self.pred_x, self.pred_y, self.pred_angle)
			return self._wall_measurements[index].translate(self.pred_x, self.pred_y, self.pred_angle)

	def get_pred_segments(self, segment_number=-1, dedensified=False):
		if dedensified:
			if segment_number == -1:
				return list(list(p.translate(self.pred_x, self.pred_y, self.pred_angle) for p in s if p in self._dedensified_measurements) for s in self._segments)
			s = self._segments[segment_number]
			return list(p.translate(self.pred_x, self.pred_y, self.pred_angle) for p in s if p in self._dedensified_measurements)
		else:
			if segment_number == -1:
				return list(list(p.translate(self.pred_x, self.pred_y, self.pred_angle) for p in s) for s in self._segments)
			s = self._segments[segment_number]
			return list(p.translate(self.pred_x, self.pred_y, self.pred_angle) for p in s)

	def get_pred_walls(self, index=-1):
		if index == -1:
			return list(w.translate(self.pred_x, self.pred_y, self.pred_angle) for w in self._relative_walls)
		return self._relative_walls[index].translate(self.pred_x, self.pred_y, self.pred_angle)

	def get_pred_wall(self, index):
		return self._relative_walls[index].translate(self.pred_x, self.pred_y, self.pred_angle)

	def get_relative_points(self, dedensified=False):
		if dedensified:
			return self._dedensified_measurements
		else:
			return self._wall_measurements

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
		with open('./node%d.txt' % self.number, 'w') as f:
			for m in self._wall_measurements:
				f.write('%f\n' % m.r())


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

	def dedensify(self, min_distance_between_nodes):
		self._dedensified_measurements = []
		for m in self._wall_measurements:
			if not any(m.distance_to(n) < min_distance_between_nodes for n in self._dedensified_measurements):
				self._dedensified_measurements.append(m)

	def get_relative_walls(self):
		return self._relative_walls

	def wall_count(self): return len(self._relative_walls)

	def segment(self):
		self._segments = LS2D.LS2D(self._wall_measurements)
		self._relative_walls = []
		for s in self._segments:
			self._relative_walls.append(Wall(*odr_line(s)))
		self.pred_rotation()

	def get_distance_to_node(self, node):
		for n in self._node_measurements:
			if n[1] is node: return n[0]
		return None

	def pred_distance_to_node(self, other):
		return math.hypot(self.pred_x - other.pred_x, self.pred_y - other.pred_y)

	def get_angle_to_node(self, node):
		return angle_to(self.pred_x, self.pred_y, node.pred_x, node.pred_y)

	def pred_rotation(self):
		print('Cleaning up room map.')
		rate = ROTATE_SPEED
		round = ROTATE_ROUNDS
		n=1
		while True:
			#draw([self], self.get_pred_walls(), quickclose=True)
			score = sum(1 / (n + dist_from_orthogonal(w.perpt())) for w in self.get_pred_walls())
			self.pred_angle += rate
			if sum(1 / (n + dist_from_orthogonal(w.perpt())) for w in self.get_pred_walls()) > score:
				continue
			else:
				self.pred_angle -= rate

			self.pred_angle -= rate
			if sum(1 / (n + dist_from_orthogonal(w.perpt())) for w in self.get_pred_walls()) > score:
				continue
			else:
				self.pred_angle += rate

			round -= 1
			rate /= 2
			if round == 0:
				break

	def get_pt_percentile(self, percentile, key):
		points = sorted(self.get_pred_points(), key=key)
		return key(points[int(percentile * len(points) / 100)])

	def sync_locations2(self, other, myindex0, myindex1, otherindex0, otherindex1):
		other.pred_x, other.pred_y, = 0, 0
		myintersect = self.get_pred_walls(myindex0).intersection(self.get_pred_walls(myindex1))

		r0 = restrict_to_unit_circle(other.pred_angle + self.get_pred_walls()[myindex0].perpt() - other.get_pred_walls()[otherindex0].perpt())
		r1 = restrict_to_unit_circle(other.pred_angle + self.get_pred_walls()[myindex1].perpt() - other.get_pred_walls()[otherindex1].perpt())
		#other.pred_angle = restrict_to_unit_circle(r0 + restrict_to_unit_circle(r1 - r0) / 2)
		otherintersect = other.get_pred_walls(otherindex0).intersection(other.get_pred_walls(otherindex1))
		other.pred_x += myintersect.x() - otherintersect.x()
		other.pred_y += myintersect.y() - otherintersect.y()