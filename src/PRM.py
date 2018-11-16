#!/usr/bin/env python
from graphs import *
from nn import nearestNeighbor
from piano_control import PianoControl
from time import sleep

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
                if SE3.check_collide(node.data, neighbor.data):
                    self.roadmap.addNeighbor(node, neighbor, cost)
                    self.roadmap.addNeighbor(neighbor, node, cost)
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
                if SE3.check_collide(node.data, neighbor.data):
                    self.roadmap.addNeighbor(node, neighbor, cost)
                    self.roadmap.addNeighbor(neighbor, node, cost)

if __name__ == '__main__':
    k = 3

    map = PRM()
    nn = map.build_roadmap(k, samples = 50)
    start = Node(SE3.get_random_state(ground=True))
    goal = Node(SE3.get_random_state(ground=True))
    map.add_points(start, goal, nn, k)
    print start, goal

    path = map.roadmap.AStarPath(start, goal)
    path_s = []
    for s in path:
        path_s.append(str(s))
    print path_s

    rocketPiano = PianoControl()
    rocketPiano.set_position(start.data)
    rocketPiano.set_steering_angle(start.data.q)
    print start.data
    for state in path:
        sleep(1)
        print state
        rocketPiano.interpolate(state)
