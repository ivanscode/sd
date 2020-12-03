from Math_Functions import *
import Node
from Drawer import draw

WALL_FUSE_ANGLE_TOLERANCE = .3
MAX_NODE_ROTATION = .75
WALL_FUSE_DISTANCE_TOLERANCE = .2
THROW_OUT_DISTANCE_BETWEEN_NODES = 15
GRAVITATION_RATE = .1
GRAVITATION_ROUNDS = 4
MIN_DISTANCE_BETWEEN_MEASUREMENTS = .3
GRAV_MIN_DIST_BETWEEN = .1
ORTHO_TOLERANCE = .1

class SLAM:
	def __init__(self, nodes):
		self.nodes = nodes
		self.angle_tolerance = WALL_FUSE_ANGLE_TOLERANCE
		self.distance_tolerance = WALL_FUSE_DISTANCE_TOLERANCE

	def get_pred_walls(self, nodelim=0, fuse=True):
		walls = []
		for n in (self.nodes[0:nodelim] if nodelim else self.nodes):
			walls += n.get_pred_walls()
		if not fuse:
			return walls
		# Fuse walls
		changed = True
		while changed:
			changed = False
			new_walls = []
			while walls:
				w = walls.pop(0)
				for w2 in walls:
					f = w.fuse(w2, self.distance_tolerance, self.angle_tolerance)
					if not f: continue
					w.fuse(w2, self.distance_tolerance, self.angle_tolerance)
					new_walls.append(f)
					walls.remove(w2)
					changed = True
					break
				else: new_walls.append(w)
			walls = new_walls
		return walls

	def score(self, nodelim=0, scoremin=0):
		nodes = self.nodes[0:nodelim] if nodelim else self.nodes
		points = []
		for n in nodes:
			points += n.get_pred_points(dedensified=True)
		dtotal = 0
		p = points.pop(0)
		while points:
			#if scoremin != 0 and -dtotal < scoremin: return None
			minp = min(points,key=lambda x: p.distance_to(x))
			points.remove(minp)
			dtotal += p.distance_to(minp)
			p = minp
		return self.corner_or_connect_count()#-dtotal
		#walls = []
		#for n in nodes:
		#	walls += n.get_pred_walls()
		#collision_count = 0
		#for i in range(0, len(walls) - 1):
		#	for j in range(i + 1, len(walls)):
		#		if walls[i].does_touch(walls[j]) and not walls[i].fuse(walls[j], WALL_FUSE_DISTANCE_TOLERANCE, WALL_FUSE_ANGLE_TOLERANCE):
		#			collision_count += 1
		return -self.gtot(1)#self.corner_or_connect_count()####self.corner_or_connect_count()#-dtotal### #- self.intersect_count()#- dtotal  # - 100 * len(self.get_pred_walls())  - 100 * collision_count -dtotal - 1000 * len(self.get_pred_walls()) - 1000 * collision_count -self.gtot(1)

	def find_best_rotation(self, nodes, best_score=0):
		PRINT_DEBUG = False
		# Base case
		if len(nodes) <= 1: return self.score(scoremin=best_score), []

		# Inductive case
		n0, n1 = nodes[0], nodes[1]
		best_combo = None
		best_moves = None

		for n0wall0 in range(n0.wall_count() - 1):
			if len(nodes) == len(self.nodes): print('%d/%d' % (n0wall0, n0.wall_count()))
			n0w0 = n0.get_pred_walls(n0wall0)
			if dist_from_orthogonal(n0w0.perpt()) > ORTHO_TOLERANCE: continue
			for n0wall1 in range(n0wall0 + 1, n0.wall_count()):
				n0w1 = n0.get_pred_walls(n0wall1)
				if dist_from_orthogonal(n0w1.perpt()) > ORTHO_TOLERANCE: continue
				if anglediff_abs(n0w0.perpt(), n0w1.perpt()) < ORTHO_TOLERANCE: continue
				for n1wall0 in range(n1.wall_count() - 1):
					n1w0 = n1.get_pred_walls(n1wall0)
					if dist_from_orthogonal(n1w0.perpt()) > ORTHO_TOLERANCE: continue
					for n1wall1 in range(n1wall0+1, n1.wall_count()):
						n1w1 = n1.get_pred_walls(n1wall1)
						if dist_from_orthogonal(n1w1.perpt()) > ORTHO_TOLERANCE: continue
						if anglediff_abs(n1w0.perpt(), n1w1.perpt()) < ORTHO_TOLERANCE: continue
						if PRINT_DEBUG: print('Attempting to fuse node %d walls %d, %d and node %d walls %d, %d' % (n0.number, n0wall0, n0wall1, n1.number, n1wall0, n1wall1))
						try:
							n0.sync_locations2(n1, n0wall0, n0wall1, n1wall0, n1wall1)
						except:
							if PRINT_DEBUG: print('Location sync failed.')
							continue

						if abs(n1.pred_angle) > MAX_NODE_ROTATION:
							continue
						if n0.pred_distance_to_node(n1) > THROW_OUT_DISTANCE_BETWEEN_NODES:
							if PRINT_DEBUG: print('\tFailed. TO distance.')
							continue
						draw(self.nodes, self.get_pred_walls(), quickclose=True)

						#if not n0.get_pred_wall(n0wall0).fuse(n1.get_pred_wall(n1wall0), self.distance_tolerance, self.angle_tolerance):
						#	if PRINT_DEBUG: print('\tFailed. Fuse1 fail.')
						#	continue
						#if not n0.get_pred_wall(n0wall1).fuse(n1.get_pred_wall(n1wall1), self.distance_tolerance, self.angle_tolerance):
						#	if PRINT_DEBUG: print('\tFailed. Fuse2 fail.')
						#	continue

						score, moves = self.find_best_rotation(nodes[1:], best_score=best_score)
						if score is None:
							if PRINT_DEBUG: print('Failed down the line, no score generated.')
							continue
						if best_score == 0 or score > best_score:
							best_score = score
							best_combo = ((n0wall0, n0wall1), (n1wall0, n1wall1))
							best_moves = moves

		if best_combo is None:
			if PRINT_DEBUG: print('No valid rotation possible. No score generated.')
			return None, []

		return best_score, [best_combo] + best_moves

	def apply_moves(self, nodes, moves):
		for i in range(0, len(nodes) - 1):
			nodes[i].sync_locations2(nodes[i+1], *moves[i][0], *moves[i][1])

	def solve_nodes(self):
		print('Assembling room map.')
		for n in self.nodes:
			n.dedensify(MIN_DISTANCE_BETWEEN_MEASUREMENTS)
		self.angle_tolerance = WALL_FUSE_ANGLE_TOLERANCE
		self.distance_tolerance = WALL_FUSE_DISTANCE_TOLERANCE

		best_list, best_score, best_moves = None, None, None
		for i in range(0, len(self.nodes)):
			score, moves = self.find_best_rotation(self.nodes)
			if best_score is None or score > best_score:
				best_list = self.nodes
				best_score = score
				best_moves = moves
			break
			self.nodes = self.nodes[1:] + [self.nodes[0]]

		moves = best_moves
		self.nodes = best_list

		while not moves:
			print('Failed to find valid map. Increasing tolerance.')
			self.angle_tolerance *= 1.1
			self.distance_tolerance *= 1.1
			score, moves = self.find_best_rotation(self.nodes)

		self.apply_moves(self.nodes, moves)
		print('Assembled room map.')

		distances = []
		for i in range(0, len(self.nodes) - 1):
			for j in range(i + 1, len(self.nodes)):
				distances.append((self.nodes[i].number, self.nodes[j].number, self.nodes[i].pred_distance_to_node(self.nodes[j])))
		return distances

	def solve_single(self, do_flips=None):
		P = 2
		best_flip = None
		best_score = 0

		xfunc = lambda x: x.x()
		yfunc = lambda x: x.y()

		for flips in [(0, 0), (0, 1), (1, 0), (1, 1)]:
			if do_flips and any(do_flips[i] != flips[i] for i in (0, 1)):
				continue
			for i in range(1, len(self.nodes)):
				p = 100 - P if flips[i-1] else P
				self.nodes[i].pred_x += self.nodes[0].get_pt_percentile(p, xfunc) - self.nodes[i].get_pt_percentile(p, xfunc)
				self.nodes[i].pred_y += self.nodes[0].get_pt_percentile(p, yfunc) - self.nodes[i].get_pt_percentile(p, yfunc)
			score = self.score()
			if best_flip is None or score > best_score:
				best_score = score
				best_flip = flips

		return best_flip, best_score

	def solve_multi(self):
		for n in self.nodes:
			n.dedensify(MIN_DISTANCE_BETWEEN_MEASUREMENTS)
		best_list, best_score, best_flips = None, None, None
		for i in range(0, len(self.nodes)):
			flips, score  = self.solve_single()
			if best_score is None or score > best_score:
				best_list = self.nodes
				best_score = score
				best_flips = flips
			self.nodes = self.nodes[1:] + [self.nodes[0]]

		self.nodes = best_list
		self.solve_single(best_flips)
		distances = []
		for i in range(0, len(self.nodes) - 1):
			for j in range(i + 1, len(self.nodes)):
				distances.append((self.nodes[i].number, self.nodes[j].number, self.nodes[i].pred_distance_to_node(self.nodes[j])))
		return distances

	def solve(self):
		print('Assembling room map.')
		for n in self.nodes:
			n.dedensify(MIN_DISTANCE_BETWEEN_MEASUREMENTS)
		self.angle_tolerance = WALL_FUSE_ANGLE_TOLERANCE
		self.distance_tolerance = WALL_FUSE_DISTANCE_TOLERANCE

		P = 2
		flip_p = [0] * len(self.nodes)
		#best_flips =
		for i in range(0, len(self.nodes)):
			for j in range(0, len(self.nodes) - 1):
				self.nodes[j+1].pred_x += self.nodes[j].get_pt_percentile(P, xfunc) - self.nodes[j+1].get_pt_percentile(P, xfunc)
				self.nodes[j+1].pred_y += self.nodes[j].get_pt_percentile(P, yfunc) - self.nodes[j+1].get_pt_percentile(P, yfunc)
			score = self.gtot(1)

		distances = []
		for i in range(0, len(self.nodes) - 1):
			for j in range(i + 1, len(self.nodes)):
				distances.append((self.nodes[i].number, self.nodes[j].number, self.nodes[i].pred_distance_to_node(self.nodes[j])))
		return distances

	def gravitate(self):
		print('Cleaning up room map.')
		g_round = 0
		grav_rate = GRAVITATION_RATE
		last_g_total = 0
		while True:
			draw(self.nodes, self.get_pred_walls(), quickclose=True)
			nodes = list({
				'x' : 0,
				'y' : 0,
				't' : 0,
			} for n in self.nodes)
			g_total = 0
			for i in range(0, len(nodes) - 1):
				for j in range(i + 1, len(nodes)):
					for p0 in self.nodes[i].get_pred_points(dedensified=True):
						for p1 in self.nodes[j].get_pred_points(dedensified=True):
							xdiff = p0.x() - p1.x()
							ydiff = p0.y() - p1.y()
							dist = math.hypot(xdiff, ydiff)
							g_total -= 1 / (dist + .4) ** 2
							xpush = xdiff / (dist + .4) ** 3
							ypush = ydiff / (dist + .4) ** 3
							nodes[i]['x'] -= xpush
							nodes[j]['x'] += xpush
							nodes[i]['y'] -= ypush
							nodes[j]['y'] += ypush
							nodes[i]['t'] -= xpush * -(p0.y() - self.nodes[i].pred_y) + ypush * (p0.x() - self.nodes[i].pred_x)
							nodes[j]['t'] += xpush * -(p1.y() - self.nodes[j].pred_y) + ypush * (p1.x() - self.nodes[j].pred_x)
			if last_g_total != 0 and g_total > last_g_total:
				print('Round %d/%d. Score: %.2f' % (g_round + 1, GRAVITATION_ROUNDS, -last_g_total))
				last_g_total = 0
				grav_rate /= 2
				g_round += 1
				if g_round == GRAVITATION_ROUNDS:
					break
				continue
			last_g_total = g_total
			maxchange = max(max((abs(n['x']), abs(n['y']), abs(n['t']))) for n in nodes)
			for i in range(0, len(nodes)):
				self.nodes[i].pred_x += nodes[i]['x'] / maxchange * grav_rate
				self.nodes[i].pred_y += nodes[i]['y'] / maxchange * grav_rate
				self.nodes[i].pred_angle += nodes[i]['t'] / maxchange * grav_rate

	def gtot(self, resolution):
		g_total = 0
		for i in range(0, len(self.nodes)):
			for j in range(0, len(self.nodes)):
				if i == j:
					continue
				for p0 in range(0, len(self.nodes[i].get_relative_points(dedensified=True)), resolution):
					p0 = self.nodes[i].get_pred_points(p0, dedensified=True)
					for p1 in range(0, len(self.nodes[j].get_relative_points(dedensified=True)), resolution):
						p1 = self.nodes[j].get_pred_points(p1, dedensified=True)
						xdiff = p0.x() - p1.x()
						ydiff = p0.y() - p1.y()
						dist = math.hypot(xdiff, ydiff)
						g_total -= 1 / (.3 + dist ** 2)

		print('\tResolution %d total g: %f' % (resolution, g_total * resolution ** 2))
		return g_total * resolution ** 2

	def corner_or_connect_count(self):
		wallsets = []
		for n in self.nodes:
			wallsets.append(n.get_pred_walls())
		count = 0
		for i in range(len(wallsets) - 1):
			wallset = wallsets[i]
			for w0 in wallset:
				found = False
				for j in range(i + 1, len(wallsets)):
					otherset = wallsets[j]
					for w1 in otherset:
						if w0.forms_corner_or_straight(w1, WALL_FUSE_ANGLE_TOLERANCE, WALL_FUSE_DISTANCE_TOLERANCE):
							count += 1
							found = True
							break
					if found:
						break
		return count

	def intersect_count(self):
		wallsets = []
		for n in self.nodes:
			wallsets.append(n.get_pred_walls())
		count = 0
		for wallset in wallsets:
			for otherset in wallsets:
				if wallset == otherset:
					continue
				for w0 in wallset:
					for w1 in otherset:
						if not w0.forms_corner_or_straight(w1, WALL_FUSE_ANGLE_TOLERANCE, WALL_FUSE_DISTANCE_TOLERANCE) and w0.does_touch(w1):
							count += 1
							break
		return count

def score(n0, n1):
	n0.dedensify(MIN_DISTANCE_BETWEEN_MEASUREMENTS)
	n1.dedensify(MIN_DISTANCE_BETWEEN_MEASUREMENTS)
	s = SLAM([n0, n1])
	return s.solve_single((0,0))[1]

def get_worst(nodes, existing):
	if not existing:
		return nodes[0]
	worst = nodes[0]
	worst_score = None
	for n in nodes:
		s = max(score(n, e) for e in existing)
		if worst_score is None or s < worst_score:
			worst_score = s
			worst = n
	return worst

if __name__ == '__main__':
	node0 = Node.Node(0,0,0,0)
	node1 = Node.Node(1,0,0,1)
	node2 = Node.Node(1,1,0,2)
	nodes = [node0, node1, node2]
	for n in nodes:
		n.sim_scan(nodes, [])

	s = SLAM(nodes)
	#s.triangulate(1)