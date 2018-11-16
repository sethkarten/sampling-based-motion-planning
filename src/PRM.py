#!/usr/bin/env python
from graphs import *
from nn import nearestNeighbor

class PRM:
    def __init__(self):
        self.roadmap = Graph()

    def build_roadmap(self, k, dist=SE3.distance, samples=100):
        nng = nearestNeighbor()
        self.roadmap = Graph()
        for i in range(samples):
            new_state = SE3.get_random_state()
            self.roadmap.addVertex(Node(new_state))
            nng.addPoint(new_state)
        nng.buildTree()
        for node in self.roadmap.graph.values():
            neighbors = nng.query_k_nearest([node.data.unpack()], k)[0]
            for j in range(1, k):
                # Ignore edge to self
                neighbor = SE3.repack(nng.values[neighbors[j]])
                cost = dist(node.data, neighbor)
                self.roadmap.addNeighbor(node, neighbor, cost)

if __name__ == '__main__':
    map = PRM()
    map.build_roadmap(10)
