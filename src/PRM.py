#!/usr/bin/env python
from graphs import *
from nn import nearestNeighbor

class PRM:
    def __init__(self):
        self.roadmap = Graph()

    def build_roadmap(self, k, dist=SE3.distance, samples=100):
        nng = nearestNeighbor()
        self.roadmap = Graph()
        list = []
        for i in range(samples):
            new_state = SE3.get_random_state()
            self.roadmap.addVertex(Node(new_state))
            list.append(new_state)
            nng.addPoint(new_state)
        nng.buildTree()
        for node in self.roadmap.graph.values():
            neighbors,distances = nng.query_k_nearest(node.data, k)
            print neighbors
            for j in range(0,len(neighbors)):
                neighbor = nng.values[neighbors[j]]
                print neighbor
                cost = distances[j]
                self.roadmap.addNeighbor(node, neighbor, cost)

if __name__ == '__main__':
    map = PRM()
    map.build_roadmap(10)
