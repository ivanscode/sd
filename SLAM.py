from Math_Functions import *
import Node

WALL_FUSE_ANGLE_TOLERANCE = .1
WALL_FUSE_DISTANCE_TOLERANCE = 1

from Drawer import draw

class SLAM:
	def __init__(self, nodes):
		self.nodes = nodes
		self.angle_tolerance = WALL_FUSE_ANGLE_TOLERANCE
		self.distance_tolerance = WALL_FUSE_DISTANCE_TOLERANCE

	def triangulate(self, orientation):
		self.nodes[0].pred_x = 0
		self.nodes[0].pred_y = 0
		self.nodes[0].pred_angle = 0

		self.nodes[1].pred_x = self.nodes[0].get_distance_to_node(self.nodes[1])
		self.nodes[1].pred_y = 0
		self.nodes[1].pred_angle = 0

		p0, p1 = triangulate(0, 0, self.nodes[1].pred_x, 0, self.nodes[0].get_distance_to_node(self.nodes[2]), self.nodes[1].get_distance_to_node(self.nodes[2]))

		if p0[1] > 0 and orientation:
			self.nodes[2].pred_x = p0[0]
			self.nodes[2].pred_y = p0[1]
		else:
			self.nodes[2].pred_x = p1[0]
			self.nodes[2].pred_y = p1[1]

		# Do the triangulate thing for further nodes. Instead of checking orientation, see if the distance to node 2 checks out.

		for n in nodes:
			print('Placed node %d at position: (%f, %f)' % (n.number, n.pred_x, n.pred_y))

	def get_pred_walls(self):
		walls = []
		for n in self.nodes:
			walls += n.get_pred_walls()

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
					new_walls.append(f)
					walls.remove(w2)
					changed = True
					break
				else: new_walls.append(w)
			walls = new_walls

		#draw([], walls)
		return walls

	def score(self):
		walls = self.get_pred_walls()
		#draw(self.nodes, walls)
		return -len(walls) * 10000 - sum(w.length() for w in walls)

	def find_best_rotation(self, nodes, prev_node=None, prev_index=-1, this_index=-1):
		# Base case
		if len(nodes) <= 1: return self.score(), []

		# Inductive case
		n0, n1 = nodes[0], nodes[1]
		best_score = 0
		best_combo = None
		best_moves = None
		for n0wall in range(0, n0.wall_count()):
			for n1wall in range(0, n1.wall_count()):
				for orientation in range(0, 2):
					#print('Attempting to fuse node 0 wall %d and node 1 wall %d with orientation %d' % (n0wall, n1wall, orientation))
					#print(nodes)
					n0.sync_rotations(n1, n0wall, n1wall, orientation)
					w0 = n0.get_pred_wall(n0wall)
					w1 = n1.get_pred_wall(n1wall)
					#draw(self.nodes, self.get_pred_walls(), quickclose=True)
					if not w0.fuse(w1, self.distance_tolerance, self.angle_tolerance):
						#print('\tFailed. Could not fuse walls.')
						continue
					if prev_node and not prev_node.get_pred_wall(prev_index).fuse(n0.get_pred_wall(this_index), self.distance_tolerance, self.angle_tolerance):
						#print('\tFailed. Broke previous wall fusion.')
						continue
					print('Fused node 0 wall %d and node 1 wall %d with orientation %d' % (n0wall, n1wall, orientation))

					score, moves = self.find_best_rotation(nodes[1:], prev_node=n0, prev_index=n0wall, this_index=n1wall)
					if score is None:
						print('Failed down the line, no score generated.')
						continue
					print('Score for this orientation: %f. Moves: %s' % (score, str(moves)))
					if best_combo is None or score > best_score:
						best_score = score
						best_combo = (n0wall, n1wall, orientation)
						best_moves = moves

		if best_combo is None:
			print('No valid rotation possible. No score generated.')
			return None, []

		#score, moves = self.find_best_rotation(nodes[1:], prev_node=n0, prev_index=best_combo[0], this_index=best_combo[1])
		return best_score, [best_combo] + best_moves

	def rotate_nodes(self):
		self.angle_tolerance = WALL_FUSE_ANGLE_TOLERANCE
		self.distance_tolerance = WALL_FUSE_DISTANCE_TOLERANCE
		score, moves = self.find_best_rotation(self.nodes)
		while not moves:
			print('Failed to find valid rotation. Increasing tolerance.')
			self.angle_tolerance *= 2
			self.distance_tolerance *= 2
			score, moves = self.find_best_rotation(self.nodes)
		print(moves)
		for i in range(0, len(self.nodes) - 1):
			self.nodes[i].sync_rotations(self.nodes[i+1], *moves[i])
		draw([],self.get_pred_walls())

if __name__ == '__main__':
	node0 = Node.Node(0,0,0,0)
	node1 = Node.Node(1,0,0,1)
	node2 = Node.Node(1,1,0,2)
	nodes = [node0, node1, node2]
	for n in nodes:
		n.sim_scan(nodes, [])

	s = SLAM(nodes)
	s.triangulate(1)