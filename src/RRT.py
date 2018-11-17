#!/usr/bin/env python
from graphs import *
from nn import nearestNeighbor
from ackermann_control import AckermannControl
from time import sleep
import sys, random

from matplotlib import collections as mc, pyplot as plt
import pylab as pl
import numpy as np

class RRT:
    def __init__(self, bot, q_start, q_goal):
        self.roadmap = Graph()
        self.mouseBot = bot
        self.nng = nearestNeighbor(se2 = True)
        self.i = 0

        q_start_node = Node(q_start)
        self.start = q_start_node
        q_goal_node = Node(q_goal)
        self.goal = q_goal_node

        self.roadmap.addVertex(q_start_node)
        self.nng.addPoint(q_start)
        self.nng.buildTree()

    def build(self, samples=100):
        for i in range(samples):
            q_rand = SE2.get_random_state()
            self.extend(q_rand)
            #print self.start.neighbors
            self.i+=1
            print self.i
        return self.connect()

    def extend(self, q_rand):
        q_near = self.nng.query_k_nearest(q_rand, 5)
        if len(q_near) == 5:
            q_near = q_near[random.randint(0,4)][0][0]
        else:
            q_near = q_near[0][0]

        print q_near
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
        return new_node

    def connect(self):
        q_goal = self.goal.data
        self.roadmap.addVertex(self.goal)
        iter = 0
        while True:
            q_new = self.extend(q_goal).data
            if SE2.euclid_dist(q_new, q_goal) < 1.0:
                self.goal = self.roadmap.graph[str(q_new)]
                return True
            iter += 1
            if iter > 5:
                break
        return False

    @staticmethod
    def merge(T1, T2, attempts=30):
        for i in range(attempts):
            q_rand = SE2.get_random_state()
            q_new1 = T1.extend(q_rand)
            q_new2 = T2.extend(q_new1.data)
            cost = SE2.distance(q_new1.data, q_new2.data)
            if cost < 0.5:
                T1.roadmap.addNeighbor(q_new1, q_new2, cost)
                T1.roadmap.addNeighbor(q_new2, q_new1, cost)
                for node in T2.roadmap.graph.values():
                    if node.data == start.data or node.data == goal.data:
                        continue
                    T1.roadmap.addVertex(node)
                return T1
            else:
                T1, T2 = T2, T1
        return None

    def print_roadmap(self, astar=False, path=None):
        Vx = []
        Vy = []
        E = []
        for node in self.roadmap.graph.values():
            Vx.append(node.data.X)
            Vy.append(node.data.Y)
            for n in node.neighbors:
                n = n.neighbor
                E.append([(node.data.X, node.data.Y), (n.data.X, n.data.Y)])
        fig, ax = pl.subplots()

        lc = mc.LineCollection(E, color='red')
        ax.add_collection(lc)
        plt.scatter(Vx,Vy,color='blue')

        if astar:
            Vx_a = []
            Vy_a = []
            E_a = []
            for i in range(len(path)):
                s = path[i]
                Vx_a.append(s.X)
                Vy_a.append(s.Y)
                if i != len(path)-1:
                    E.append([(s.X, s.Y), (path[i+1].X, path[i+1].Y)])
            lc1 = mc.LineCollection(E_a, color='green', linewidths=3)
            ax.add_collection(lc1)
            plt.scatter(Vx_a, Vy_a, color='green', linewidths=3)

        fig.tight_layout()
        plt.show()



if __name__ == "__main__":
    mouseBot = AckermannControl()
    q_start=SE2(-8, -6.5, 3.14/2.0)
    q_goal=SE2(9, 5.5, 3*3.14/2.0)
    map = RRT(mouseBot, q_start, q_goal)
    map1 = RRT(mouseBot, q_goal, q_start)
    while True:
        if map.build():
            break
        if map1.build():
            map = map1
            map.start, map.goal = map.goal, map.start
            break
        map2 = RRT.merge(map, map1)
        if map2 != None:
            map = map2
            break
    map.print_roadmap()
    print map.start.neighbors
    print map.goal.neighbors
    raw_input('Start A*?')
    path = map.roadmap.AStarPath(map.start, map.goal, h=SE2.distance)
    if path == None:
        print None
    path.insert(0, map.start.data)
    map.print_roadmap(astar=True, path=path)

    path_s = []
    for s in path:
        path_s.append(str(s))
    print path_s

    for q in path:
        print q
        mouseBot.set_state(q.X, q.Y, q.theta, vx=q.vx, vy=q.vy, vs=q.vs)
        sleep(1)
