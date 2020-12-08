from Math_Functions import *
import Drawer

class Wall:
	def __init__(self, p0, p1):
		self.p0 = p0
		self.p1 = p1

	def x0(self): return self.p0.x()

	def x1(self): return self.p1.x()

	def y0(self): return self.p0.y()

	def y1(self): return self.p1.y()

	def r0(self): return self.p0.r()

	def r1(self): return self.p1.r()

	def t0(self): return self.p0.t()

	def t1(self): return self.p1.t()

	def perpr(self): return self.perp().r()

	def perpt(self): return self.perp().t()

	def maxx(self): return max(self.x0(), self.x1())

	def maxy(self): return max(self.y0(), self.y1())

	def minx(self): return min(self.x0(), self.x1())

	def miny(self): return min(self.y0(), self.y1())

	def angle(self): return angle_to(self.x0(), self.y0(), self.x1(), self.y1())

	def length(self): return self.p0.distance_to(self.p1)

	def min_distance_to_wall(self, other, maxval):
		possibilities = [maxval + 1]
		v = distance_to_two_sided(self.x0(), self.y0(), other.perpt(), other.x0(), other.y0(), other.x1(), other.y1())
		if v: possibilities.append(v)
		v = distance_to_two_sided(self.x1(), self.y1(), other.perpt(), other.x0(), other.y0(), other.x1(), other.y1())
		if v: possibilities.append(v)
		possibilities.append(self.min_corner_distance(other))
		return min(possibilities)

	def min_corner_distance(self, other):
		return min([
			self.p0.distance_to(other.p0),
			self.p0.distance_to(other.p1),
			self.p1.distance_to(other.p0),
			self.p1.distance_to(other.p1)
		])

	def angle_between(self, other):
		angle = abs(restrict_to_unit_circle(self.perpt() - other.perpt()))
		if angle > math.pi / 2:
			angle = math.pi - angle
		return angle

	def forms_corner_or_straight(self, other, angle_tolerance, distance_tolerance):
		angle = self.angle_between(other)
		if angle < angle_tolerance:
			return self.min_distance_to_wall(other, distance_tolerance) < distance_tolerance
		if abs(angle - math.pi / 2) < angle_tolerance:
			return self.min_corner_distance(other) < distance_tolerance
		return False

	def perp(self):
		m = (self.y1() - self.y0()) / (self.x1() - self.x0())
		b = self.y0() - self.x0() * m
		return Point(-b / (m + 1 / m), b / (m*m + 1))

	def translate(self, x_offset, y_offset, angle_offset):
		return Wall(self.p0.translate(x_offset, y_offset, angle_offset), self.p1.translate(x_offset, y_offset, angle_offset))

	def fuse(self, other, distance_tolerance, angle_tolerance):
		PRINT_DEBUG = False
		if PRINT_DEBUG: print('\tFusing:')
		if PRINT_DEBUG: print('\t\t' + str(self))
		if PRINT_DEBUG: print('\t\t' + str(other))

		if self.angle_between(other) > angle_tolerance:
			if PRINT_DEBUG: print('\t\tFailed. Angle ' + str(anglediff) + ' > tolerance.')
			return None

		# Also makes sure the walls overlap
		mindist = min((self.min_distance_to_wall(other, distance_tolerance), other.min_distance_to_wall(self, distance_tolerance)))
		if mindist > distance_tolerance:
			if PRINT_DEBUG: print('\t\tFailed. Distance %f > tolerance.' % mindist)
			return None

		# I'm just going to take the outer two points and connect them. If this doesn't work, maybe try a more intelligent approach
		pts = [self.p0, self.p1, other.p0, other.p1]
		maxdist = 0
		maxpair = (self.p0, self.p1)
		for p0 in pts:
			for p1 in pts:
				d = p0.distance_to(p1)
				if d > maxdist:
					maxdist = d
					maxpair = (p0, p1)

		return Wall(*maxpair)

	def intersection(self, other):
		# https://stackoverflow.com/questions/20677795/how-do-i-compute-the-intersection-point-of-two-lines
		x1, y1, x2, y2 = self.p0.x(), self.p0.y(), self.p1.x(), self.p1.y()
		x3, y3, x4, y4 = other.p0.x(), other.p0.y(), other.p1.x(), other.p1.y()
		x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4))
		y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4))
		return Point(x, y)

	def does_touch(self, other):
		# https://stackoverflow.com/questions/20677795/how-do-i-compute-the-intersection-point-of-two-lines
		x1, y1, x2, y2 = self.p0.x(), self.p0.y(), self.p1.x(), self.p1.y()
		x3, y3, x4, y4 = other.p0.x(), other.p0.y(), other.p1.x(), other.p1.y()
		x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4))
		y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4))
		return x > min(self.x0(), self.x1()) and x < max(self.x0(), self.x1())

	def __str__(self):
		return 'Wall(Point%s, Point%s),\t\tPerpendicular point %s, r theta %s' % (str(self.p0), str(self.p1), str(self.perp()), self.perp().rthetastr())

if __name__ == '__main__':
	walls = [
		Wall(Point(0, 0), Point(.01, 5)),
		Wall(Point(.2, 0), Point(.1, 5)),
	]
	for i in range(len(walls) - 1):
		for j in range(i + 1, len(walls)):
			print(walls[i].forms_corner_or_straight(walls[j], .1, .3))
	Drawer.PIXELS_PER_METER = 50
	Drawer.draw([], walls)