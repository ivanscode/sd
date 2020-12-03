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
import threading
import Master

DEVICE_COUNT = 3 #Not used atm
NET_BASE = '192.168.1.' #Change depending on network
RL = 37 #From IP.2
RH = 40 #To IP.20

class RoomMap:
    def __init__(self, name):
        self.name = name
        self.nodes = {}
        self.pairs = {}
        self.dist = {}
        self.data = {}
        self.allnodes = False
        self.distances = []

    def scan(self, addr):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        socket.setdefaulttimeout(1)
        print('Trying {}'.format(addr))
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
                print('{} is available'.format(addr))
                time.sleep(0.5) #Need to wait for socket to reopen
                self.nodes[idx] = sm.SocketManager(addr)
                idx += 1

        #self.nodes = ("IP1 - Node 1", "IP2 - Node 2", "IP3 - Node 3")

    def pair_nodes(self):
        self.pairs = {}
        for node1 in self.nodes:
            for node2 in self.nodes:
                if node1 != node2:
                    self.pairs[f'{node1} + {node2}'] = 30

    def calculate_dist(self):
        threads = []
        for node in self.nodes:
            x = threading.Thread(target=thread_function, args=(self.nodes[node],))
            threads.append(x)
            x.start()

        for x in threads:
            x.join()
        
        for node in self.nodes:
            self.data[node] = self.nodes[node].data
            print(self.data[node])

        
def thread_function(node):
    node.collectData()


myMap = RoomMap(name="Test")

@app.route("/")
def index():
    global myMap
    global start
    start = True
    print(myMap.name)
    return render_template("public/index.html", myMap=myMap)

start = True
@app.route("/start", methods=["POST"])
def start():
    global myMap
    global start
    args = request.args

    if start:
        myMap.find_nodes()
        start = False

    pair = False
    measure = False
    slam = False

    if args.get("pair") == "true":
        myMap.pair_nodes()
        print("Pairing")
        pair = True
    if args.get("measure") == "true":
        myMap.calculate_dist()
        print("Measuring")
        pair = True
        measure = True
    if args.get("slam") == "true":
        print("Slamming")
        myMap.distances = Master.run(myMap.data)
        pair = True
        measure = True
        slam = True
        pygame.display.quit()
        #pygame.quit()
    #print(args)

    
    return render_template(
        "public/index.html", myMap=myMap, start=True,
        pair=pair, measure=measure, slam=slam
        )

@app.route("/about")
def about():
    return render_template("public/about.html")
