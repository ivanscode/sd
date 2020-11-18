import Node
import Drawer

FILES = ['ivan_room.txt']

if __name__ == "__main__":
	nodes = []
	for i in range(len(FILES)):
		n = Node.Node(0, 0, 0, i)
		n.set_measurement_file(FILES[i])
		n.segment()
		nodes.append(n)
	Drawer.draw_to_image(nodes, [], 'test.png')