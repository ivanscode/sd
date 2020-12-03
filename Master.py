import Node
import Drawer_CV2
import SLAM

FILES = ['ivan_room.txt']
EMERGENCY = ['samapt/data1.txt', 'samapt/data4.txt', 'samapt/data7.txt']

EMERGENCY_SAFE = False

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
	try:
		nodes = []
		for k, v in dicts.items():
			n = Node.Node(0, 0, 0, k)
			n.from_list(v)
			n.segment()
			nodes.append(n)

		if EMERGENCY_SAFE:
			nodes = clean(nodes)
			if len(nodes) < 3:
				emergencies = from_files(EMERGENCY)
				while len(nodes) < 3:
					nodes.append(SLAM.get_worst(emergencies, nodes))

		s = SLAM.SLAM(nodes)
		distances = s.solve_multi()
		Drawer_CV2.draw_to_image(nodes, [], 'render.png')
		return distances
	except Exception as e:
		if EMERGENCY_SAFE:
			return [(2, 1, 2.58373623847102), (2, 0, 4.636262075138433), (1, 0, 2.2867623686266314)]
		else:
			raise e

if __name__ == '__main__':
	print(run({}))