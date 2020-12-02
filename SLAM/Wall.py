from Math_Functions import *
import Drawer

class Wall:
	def __init__(self, p0, p1):
		self.p0 = p0
		self.p1 = p1

	def perp(self): return Point(self.perpx(), self.perpy())

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
		return min(possibilities)

	def perpx(self):
		m = (self.y1() - self.y0()) / (self.x1() - self.x0())
		b = self.y0() - self.x0() * m
		return -b / (m + 1 / m)

	def perpy(self):
		m = (self.y1() - self.y0()) / (self.x1() - self.x0())
		b = self.y0() - self.x0() * m
		return b / (m*m + 1)

	def translate(self, x_offset, y_offset, angle_offset):
		return Wall(self.p0.translate(x_offset, y_offset, angle_offset), self.p1.translate(x_offset, y_offset, angle_offset))

	def fuse(self, other, distance_tolerance, angle_tolerance):
		#print('Attempting to fuse walls:')
		#print('\t' + str(self))
		#print('\t' + str(other))
		anglediff = abs(restrict_to_unit_circle(self.perpt() - other.perpt()))
		if abs(anglediff) > angle_tolerance and (math.pi - anglediff) > angle_tolerance: return None

		# Also makes sure the walls overlap
		mindist = min((self.min_distance_to_wall(other, distance_tolerance), other.min_distance_to_wall(self, distance_tolerance)))
		#print('Mindist calculated: %f' % mindist)
		if mindist > distance_tolerance: return None

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

	def __str__(self):
		return 'Wall(Point%s, Point%s),\t\tPerpendicular point %s, r theta %s' % (str(self.p0), str(self.p1), str(self.perp()), self.perp().rthetastr())

if __name__ == '__main__':
	walls = [
		Wall(Point(3.498724, -4.585215), Point(6.018046, -1.868769)),
		Wall(Point(3.917032, -4.170225), Point(6.058277, -1.909813)),
	]
	for i in range(0, len(walls) - 1):
		for j in range(i+1, len(walls)):
			print(walls[i].fuse(walls[j], .1, .5))
	Drawer.PIXELS_PER_METER = 100
	Drawer.draw([], walls)