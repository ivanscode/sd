import Node
import Drawer_CV2
import SLAM

FILES = ['ivan_room.txt']

def from_files(files=None):
	if not files:
		files = FILES
	nodes = []
	for i in range(len(files)):
		n = Node.Node(0,0,0,i)
		n.set_measurement_file(files[i])
		n.segment()
		nodes.append(n)
	return nodes

def clean(nodes):
	for i in range(len(nodes) - 1, -1, -1):
		if len(nodes[i].get_relative_points()) < 500:
			nodes.pop(i)
	for i in range(len(nodes) - 1, -1, -1):
		if nodes[i].wall_count() < 10:
			nodes.pop(i)
	return nodes

def run(dicts):
	nodes = []
	for k, v in dicts.items():
		n = Node.Node(0, 0, 0, k)
		n.from_list(v)
		n.segment()
		nodes.append(n)

	nodes = clean(nodes)

	s = SLAM.SLAM(nodes)
	distances = s.solve_multi()
	Drawer_CV2.draw_to_image(nodes, [], 'app/static/img/render.png')
	return distances

if __name__ == '__main__':
	print(run({}))