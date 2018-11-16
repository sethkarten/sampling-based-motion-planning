#!/usr/bin/env python
from graphs import *
from nn import nearestNeighbor

class PRM:
    def __init__(self):
        self.roadmap = Graph()

    def build_roadmap(self, k, dist=SE.distance):
        nng = nearestNeighbor()
        self.roadmap = Graph()
        list = []
        for n in range(5000):
            new_state = SE3.get_random_state()
            self.roadmap.addVertex(Node(new_state))
            list.append(new_state)
            nng.addPoint(nng)
        nng.buildTree()
        for n in range(5000):
            node = self.roadmap[n]
            neighbors = nng.query_k_nearest(node.data, k)
            for neighbor in neighbors:
                cost = dist(node, neighbor)
                self.roadmap.addNeighbor(node, neighbor, cost)
