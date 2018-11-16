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
            neighbors, distances = nn.query_k_nearest(node.data, k)
            for neighbor, cost in zip(neighbors, distances):
                string_neigh = str(neighbor)
                neighbor = self.roadmap.graph[string_neigh]
                self.roadmap.addNeighbor(node, neighbor, cost)
        return nn

    def add_points(self, node1, node2, nn, k):
        for node in [node1, node2]:
            point = node.data
            self.roadmap.addVertex(node)
            nn.addPoint(point)
            nn.buildTree()
            neighbors, distances = nn.query_k_nearest(point, k)
            for neighbor, cost in zip(neighbors, distances):
                neighbor = self.roadmap.graph[str(neighbor)]
                self.roadmap.addNeighbor(node, neighbor, cost)

if __name__ == '__main__':
    k = 10

    map = PRM()
    nn = map.build_roadmap(k)
    start = Node(SE3.get_random_state(ground=True))
    goal = Node(SE3.get_random_state(ground=True))
    map.add_points(start, goal, nn, k)

    path = map.roadmap.AStarPath(start, goal)
    print path

    rocketPiano = PianoControl()
    rocketPiano.set_position(start)
    rocketPiano.set_steering_angle(start.q)
    old = position
    for state in path:
        rocketPiano.interpolate(state)
        sleep(1)
