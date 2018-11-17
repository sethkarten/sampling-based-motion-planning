#!/usr/bin/env python
from graphs import *
from nn import nearestNeighbor
from ackermann_control import AckermannControl
from time import sleep
import sys

from matplotlib import collections as mc, pyplot as plt
import pylab as pl
import numpy as np

class RRT:
    def __init__(self, bot):
        self.roadmap = Graph()
        self.mouseBot = bot
        self.nng = nearestNeighbor(se2 = True)
        self.i = 0

    def build(self, samples=350):
        q_start = SE2(-8, -6.5, 3.14/2.0)
        q_start_node = Node(q_start)
        self.start = q_start_node
        self.roadmap.addVertex(q_start_node)
        self.nng.addPoint(q_start)
        self.nng.buildTree()
        for i in range(samples):
            q_rand = SE2.get_random_state()
            self.extend(q_rand)
            self.i+=1
            print self.i
        self.connect()

    def extend(self, q_rand):
        q_near = self.nng.query_k_nearest(q_rand, 1)[0][0]
        #print q_near
        min_dist = sys.maxint
        # chose closest control
        for i in range(1):
            q_new = self.mouseBot.get_new_state(q_near)
            if SE2.euclid_dist(q_new, q_rand) < min_dist:
                q_new_best = q_new
        new_node = Node(q_new_best)
        self.roadmap.addVertex(new_node)
        old_node = self.roadmap.graph[str(q_near)]
        cost = SE2.distance(q_new_best, q_near)
        self.roadmap.addNeighbor(old_node, new_node, cost)
        self.roadmap.addNeighbor(new_node, old_node, cost)
        self.nng.addPoint(q_new_best)
        self.nng.buildTree()
        return q_new_best

    def connect(self):
        q_goal = SE2(9, 5.5, 0)
        q_goal_node = Node(q_goal)
        self.goal = q_goal_node
        self.roadmap.addVertex(q_goal_node)
        iter = 0
        while True:
            q_new = self.extend(q_goal)
            if SE2.euclid_dist(q_new, q_goal) < 0.5:
                self.goal = self.roadmap.graph[str(q_new)]
                return
            iter += 1
            if iter > 20:
                break
        self.build(samples=100)

    def print_roadmap(self):
        V = []
        E = []
        for node in self.roadmap.graph.items():
            V.append(node.data)
            for n in node.neighbors:
                E.append(n)



if __name__ == "__main__":
    mouseBot = AckermannControl()
    map = RRT(mouseBot)
    map.build()
    #map.print_roadmap()
    print map.start.neighbors
    print map.goal.neighbors
    raw_input('Start A*?')
    path = map.roadmap.AStarPath(map.start, map.goal, h=SE2.distance)
    if path == None:
        print None

    path_s = []
    for s in path:
        path_s.append(str(s))
    print path_s

    qs = map.start
    mouseBot.set_model_state(qs.X, qs.Y, qs.theta, vx=qs.vx, vy=qs.vy, vs=qs.vs)
    for q in path:
        print q
        mouseBot.set_model_state(q.X, q.Y, q.theta, vx=q.vx, vy=q.vy, vs=q.vs)
