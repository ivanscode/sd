import Node
import Drawer
import SLAM

FILES = ['ivan_room.txt']

def from_files():
	nodes = []
	for i in range(len(FILES)):
		n = Node.Node(0,0,0, i)
		n.set_measurement_file(FILES[i])
		n.segment()
		nodes.append(n)
	return nodes

def run(dicts):
	nodes = []
	for k, v in dicts.items():
		n = Node.Node(0, 0, 0, k)
		n.from_list(v)
		nodes.append(n)
	s = SLAM.solve_multi(nodes)
	distances = s.solve_nodes()
	Drawer.draw_to_image(nodes, [], 'render.png')
	return distances
