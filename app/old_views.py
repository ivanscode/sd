from app import app

from flask import render_template, request, redirect, jsonify, make_response, flash

from datetime import datetime

import time

@app.route("/")
def index():
    return render_template("public/index.html")


class RoomMap:
    def __init__(self, name):
        self.name = name
    
    def find_nodes(self):
        self.nodes = ("IP1 - Node 1", "IP2 - Node 2", "IP3 - Node 3")
        return self.nodes

    def pair_nodes(self):
        self.pairs = {}
        for node1 in self.nodes:
            for node2 in self.nodes:
                if node1 != node2:
                    self.pairs[f'{node1} + {node2}'] = 30



@app.route("/start", methods=["POST"])
def start():
    
    start = True

    def find_nodes():
        nodes = ("IP1 - Node 1", "IP2 - Node 2", "IP3 - Node 3")
        return nodes

    args = request.args
    pair = False
    dist = False
    measure = False
    slam = False
    if args.get("pair") == "true":
        pair = True
    if args.get("distance") == "true":
        dist = True
    if args.get("measure") == "true":
        measure = True
    if args.get("slam") == "true":
        slam = True
    print(args)

    def pair_nodes(node_list):
        node_pairs = {
            "N1,N2": 30,
            "N1,N3": 30,
            "N2,N1": 30,
            "N2,N3": 30,
            "N3,N1": 30,
            "N3,N2": 30
        }
        return node_pairs
    
    def calculate_dist(node_ip):
        dist_val = []
        for i in range(10):
            dist_val.append(i)
        return node_ip, dist_val

    def create_map(dist_val):
        time.sleep(10)
        
        return dist_val

    return render_template(
        "public/index.html", start=start, find_nodes=find_nodes,
        pair_nodes=pair_nodes, pair=pair, dist=dist, slam=slam,
        calculate_dist=calculate_dist, create_map=create_map
        )

@app.route("/about")
def about():
    return render_template("public/about.html")
