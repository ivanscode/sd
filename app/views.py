from app import app
import Drawer
import Node
import pygame

from flask import render_template, request, redirect, jsonify, make_response, flash

from datetime import datetime

import time

import SocketManager as sm
import socket
import _thread as thread

DEVICE_COUNT = 3 #Not used atm
NET_BASE = '192.168.1.' #Change depending on network
RL = 6 #From IP.2
RH = 8 #To IP.20

class RoomMap:
    def __init__(self, name):
        self.name = name
        self.nodes = ()
        self.pairs = {}
        self.dist = {}
        self.allnodes = False

    def scan(self, addr):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        socket.setdefaulttimeout(1)
        #print('Trying {}'.format(addr))
        result = s.connect_ex((addr, sm.PORT))
        if result == 0:
            return 1
        else :
            return 0

    def find_nodes(self):
        self.nodes = {}
        idx = 0
        for stem in range(RL, RH):
            addr = NET_BASE + str(stem)
            if(self.scan(addr)):
                #print('{} is available'.format(addr))
                time.sleep(0.5) #Need to wait for socket to reopen
                self.nodes[idx] = sm.SocketManager(addr)
                idx += 1

        #self.nodes = ("IP1 - Node 1", "IP2 - Node 2", "IP3 - Node 3")
        return self.nodes

    def pair_nodes(self):
        self.pairs = {}
        for node1 in self.nodes:
            for node2 in self.nodes:
                if node1 != node2:
                    self.pairs[f'{node1} + {node2}'] = 30

    def calculate_dist(self):
        self.dist = {}

        for node in self.nodes:
            data = []
            for i in range(10):
                data.append(i)
            self.dist[node] = data

        return self.dist


myMap = RoomMap(name="Test")

@app.route("/")
def index():
    global myMap
    print(myMap.name)
    return render_template("public/index.html", myMap=myMap)

@app.route("/start", methods=["POST"])
def start():
    global myMap
    
    args = request.args
    start = True
    myMap.find_nodes()

    pair = False
    measure = False
    slam = False

    if args.get("pair") == "true":
        myMap.pair_nodes()
        pair = True
    if args.get("measure") == "true":
        myMap.calculate_dist()
        measure = True
    if args.get("slam") == "true":
        FILES = ['ivan_room.txt']
        nodes = []
        for i in range(len(FILES)):
            n = Node.Node(0, 0, 0, i)
            n.set_measurement_file(FILES[i])
            n.segment()
            nodes.append(n)
        Drawer.draw_to_image(nodes, [], 'app/static/img/test.png')
        slam = True
        pygame.display.quit()
        #pygame.quit()
    print(args)

    
    return render_template(
        "public/index.html", myMap=myMap, start=start,
        pair=pair, measure=measure, slam=slam
        )

@app.route("/about")
def about():
    return render_template("public/about.html")
