#!/usr/bin/env python
from graphs import *
from nn import nearestNeighbor
from ackermann_control import AckermannControl
from time import sleep
import sys

class RRT:
    def __init__(self):
        self.roadmap = Graph()
        self.mouseBot = AckermannControl()
        self.nng = nearestNeighbor(metric=se2Dist, SE2.repack)

    def build(self, samples=50):
        q_start = SE2(-8, -6.5, 0)
        q_start_node = Node(q_start)
        self.roadmap.graph.addVertex(q_start_node)
        self.nng.addPoint(q_start)
        self.nng.buildTree()
        for i in range(len(samples)):
            q_rand = SE2.get_random_state()
            self.extend(q_rand)
        self.connect()

    def extend(self, q_rand):
        q_near = self.nn_start.query_k_nearest(q_rand, 1)
        min_dist = sys.maxint
        # chose closest control
        for i in range(10):
            q_new = self.mouseBot.get_new_state(q_rand)
            if SE2.euclid_dist(q_new, q_rand) < min_dist:
                q_new_best = q_new
        new_node = Node(q_new_best)
        self.roadmap.addVertex(new_node)
        old_node = self.roadmap.graph[str(q_near)]
        self.roadmap.addNeighbor(old_node, new_node)
        self.roadmap.addNeighbor(new_node, old_node)
        self.nng.addPoint(q_new_best)
        self.nng.buildTree()
        return q_new_best

    def connect(self):
        q_goal = SE2(9, 5.5, 0)
        while True:
            q_new = self.extend(q_goal)
            if SE2.euclid_dist(q_new, q_goal) < 0.5:
                break

if __name__ == "__main__":
    map = RRT()
    
