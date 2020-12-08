import Node
import Wall
from Math_Functions import Point
import SLAM
import Drawer

PIXELS_PER_METER = .7
BORDER_WIDTH = 100
PILLAR_RADIUS = .4


FILES = [
	'samapt/data1.txt',
	#'samapt/data2.txt',
	#'samapt/data3.txt',
	'samapt/data4.txt',
	#'samapt/data5.txt',
	#'samapt/data6.txt',
	#'samapt/data7.txt',
	'samapt/data8.txt',
	#'samapt/data9.txt',
]

def from_files(flist=None):
	if flist is None:
		flist = FILES
	nodes = []
	for i in range(len(flist)):
		n = Node.Node(0,0,0, i)
		n.set_measurement_file(FILES[i])
		n.segment()
		nodes.append(n)
	return nodes

if __name__ == '__main__':
	# Initialize nodes and walls, scan
	#nodes = from_files()
	#walls = [Wall.Wall(Point(0,0), Point(0,0))]
#
	#Drawer.draw(nodes, walls)
#
	##for n in nodes:
	##	n.pred_x = 0
	##	n.pred_y = 0
	##	n.pred_angle = 0
	##	n.real_x = 0
	##	n.real_y = 0
	##	n.real_angle = 0
	#Drawer.draw(nodes, [])
	#s = SLAM.SLAM(nodes)
	#print(s.solve_multi())
	##s.gravitate()
	#Drawer.draw(nodes, walls)
	#Drawer.draw(nodes, [])

	for i in range(3):
		i = 'samapt/data%d.txt' % i
		for j in range(3):
			j = 'samapt/data%d.txt' % (j + 3)
			for k in range(3):
				k = 'samapt/data%d.txt' % (k + 6)
				nodes = from_files([i, j, k])
				s = SLAM.SLAM(nodes)
				print(s.solve_multi())
				Drawer.draw(nodes, [], quickclose=True)