#!/usr/bin/env python
from graphs import *
from nn import nearestNeighbor
from piano_control import PianoControl

class PRM:
    def __init__(self):
        self.roadmap = Graph()

    def build_roadmap(self, k, samples=100):
        nn = nearestNeighbor()
        self.roadmap = Graph()
        for i in range(samples):
            new_state = SE3.get_random_state()
            self.roadmap.addVertex(Node(new_state))
            nn.addPoint(new_state)
        nn.buildTree()
        for node in self.roadmap.graph.values():
<<<<<<< HEAD
            neighbors, distances = nn.query_k_nearest(node.data, k)
            for j in range(len(neighbors)):
                neighbor = nng.values[neighbors[j]]
                cost = distances[j]
        return nn

    def add_points(self, node1, node2, nn, k):
        for node in [node1, node2]:
            point = node.data
            self.roadmap.addVertex(node)
            nn.addPoint(point)
            nn.buildTree()
            neighbors, distances = nn.query_k_nearest(point, k)
            for j in range(len(neighbors)):
                neighbor = SE3.repack(nn.values[neighbors[j]])
=======
            neighbors,distances = nng.query_k_nearest(node.data, k)
            print neighbors
            for j in range(0,len(neighbors)):
                neighbor = nng.values[neighbors[j]]
                print neighbor
>>>>>>> b920aad9e8201f1920246198e3591685842b7d02
                cost = distances[j]
                self.roadmap.addNeighbor(node, neighbor, cost)

if __name__ == '__main__':
    k = 10

    map = PRM()
    nn = map.build_roadmap(k)
    start = Node(SE3.get_random_state(ground=True))
    goal = Node(SE3.get_random_state(ground=True))
    map.add_points(start, goal, nn, k)

    path = map.AStarPath(start, target)
    print path

    rocketPiano = PianoControl()
    rocketPiano.interpolate(start)
    old = position
    for state in path:
        rocketPiano.interpolate(state)
        sleep(1)
