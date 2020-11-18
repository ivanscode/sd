import random
import math
from scipy.odr import Model, ODR, RealData

FP_EQUAL_TOLERANCE = .0001


class Point:
	def __init__(self, x, y):
		self._x = x
		self._y = y

	def x(self, offset=0):
		return self._x + offset

	def y(self, offset=0):
		return self._y + offset

	def r(self):
		return math.hypot(self._x, self._y)

	def t(self, offset=0):
		return restrict_to_unit_circle(math.atan2(self._y, self._x) + offset)

	def translate(self, x_offset, y_offset, angle_offset):
		return Point(x_offset + math.cos(self.t() + angle_offset) * self.r(), y_offset + math.sin(self.t() + angle_offset) * self.r())

	def distance_to(self, other):
		return math.hypot(self.x() - other.x(), self.y() - other.y())

	def __str__(self):
		return '(%f, %f)' % (self.x(), self.y())

	def rthetastr(self):
		return '(%f, %f)' % (self.r(), self.t())

def pt_thetar(theta, r):
	return Point(r * math.cos(theta), r * math.sin(theta))


def meanResiduals(pts):
	f = lambda B, x: B[0] * x + B[1]
	model = Model(f)
	data = RealData(list(p.x() for p in pts), list(p.y() for p in pts))
	odr = ODR(data, model, beta0=[0., 1.])
	out = odr.run()
	return out.sum_square / len(pts)


def odr_line(pts):
	f = lambda B, x: B[0] * x + B[1]
	model = Model(f)
	data = RealData(list(p.x() for p in pts), list(p.y() for p in pts))
	odr = ODR(data, model, beta0=[0., 1.])
	out = odr.run()
	m = out.beta[0]
	b = out.beta[1]

	if abs(m) > 1:
		# Do it by y
		maxY = max(p.y() for p in pts)
		minY = min(p.y() for p in pts)
		return Point((minY - b) / m, minY), Point((maxY - b) / m, maxY)

	else:
		# Do it by x
		maxX = max(p.x() for p in pts)
		minX = min(p.x() for p in pts)
		return Point(minX, m * minX + b), Point(maxX, m * maxX + b)


def tolerance(center, range):
	return random.uniform(-range, range) + center


def angle_to(xs, ys, xd, yd):
	"""
	Returns angle from point (xs, ys) to (xd, yd)
	"""
	return math.atan2(yd - ys, xd - xs)


def restrict_to_unit_circle(angle):
	return math.atan2(math.sin(angle), math.cos(angle))


def distance_to(xs, ys, rs, xd0, yd0, xd1, yd1):
	"""
	Returns length of line extending from point (xs, ys) along radian angle rs to wall from point (xd0, yd0) to point (xd1, yd1).
	If no intersection is found, returns -1
	"""
	# Get it to intersection of two lines
	rd = angle_to(xd0, yd0, xd1, yd1)

	angle_s = restrict_to_unit_circle(angle_to(xs, ys, xd0, yd0) - rs)
	angle_d = restrict_to_unit_circle(angle_to(xd0, yd0, xs, ys) - rd)

	if (angle_s < 0) == (angle_d < 0) or (abs(angle_s - angle_d) > math.pi):
		return None

	try:
		dist_d = abs(math.hypot(xs - xd0, ys - yd0) / math.sin(math.pi - abs(angle_s) - abs(angle_d)) * math.sin(angle_s))
	except ZeroDivisionError:
		return None

	if dist_d < math.hypot(xd1 - xd0, yd1 - yd0):
		try:
			return abs(math.hypot(xs - xd0, ys - yd0) / math.sin(math.pi - abs(angle_s) - abs(angle_d)) * math.sin(angle_d))
		except ZeroDivisionError:
			return None

	return None


def distance_to_two_sided(xs, ys, rs, xd0, yd0, xd1, yd1):
	d = distance_to(xs, ys, rs, xd0, yd0, xd1, yd1)
	if d: return d
	return distance_to(xs, ys, rs + math.pi, xd0, yd0, xd1, yd1)


def triangulate(x0, y0, x1, y1, d0, d1):
	a = math.hypot(y1 - y0, x1 - x0)
	b = d1
	c = d0

	B = math.acos((a*a + c*c - b*b) / a / c / 2)
	angle_possibilities = (angle_to(x0, y0, x1, y1) + B, angle_to(x0, y0, x1, y1) - B)

	return (x0 + c * math.cos(angle_possibilities[0]), y0 + c * math.sin(angle_possibilities[0])), \
			(x0 + c * math.cos(angle_possibilities[1]), y0 + c * math.sin(angle_possibilities[1]))


def fpeq(a, b):
	return abs( a- b) < FP_EQUAL_TOLERANCE