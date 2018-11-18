#!/usr/bin/env python
from graphs import *
from nn import nearestNeighbor
from ackermann_control import AckermannControl
from time import *
import sys, random

from matplotlib import collections as mc, pyplot as plt
import pylab as pl
import numpy as np



class RRT:
    def __init__(self, bot, q_start, q_goal, greedy=False):
        self.roadmap = Graph()
        self.mouseBot = bot
        self.nng = nearestNeighbor(se2 = True)
        self.i = 0

        self.greedy = greedy

        q_start_node = Node(q_start)
        self.start = q_start_node
        q_goal_node = Node(q_goal)
        self.goal = q_goal_node

        self.roadmap.addVertex(q_start_node)
        self.nng.addPoint(q_start)
        self.nng.buildTree()

    def build(self, samples=125):
        for i in range(samples):

            q_rand = SE2.get_random_state()
            if i % 3  == 0:
                q_rand = SE2.get_random_state(greedy=self.greedy, goal=self.goal.data)
            #if i % 15 == 0:
            #    q_rand = SE2.get_random_state(greedy=True, goal=self.start.data)
            #print q_rand
            new_node = self.extend(q_rand)
            if SE2.distance(new_node.data, self.goal.data) < 1.5:
                #print new_node.data
                self.goal = new_node
                #print 'First solution', self.i
                return True

            #print self.start.neighbors
            self.i+=1
            print self.i
        return self.connect()

    def extend(self, q_rand):
        q_near = self.nng.query_k_nearest(q_rand, 1)
        if len(q_near) == 5:
            q_near = q_near[random.randint(0,4)][0][0]
        else:
            q_near = q_near[0][0]

        #print q_near
        min_dist = sys.maxint
        # chose closest control
        closest = 1
        if self.greedy:
            closest = 2
        for i in range(closest):
            q_new = self.mouseBot.get_new_state(q_near)
            test_q = q_rand
            if self.greedy:
                test_q = self.goal.data
            if SE2.euclid_dist(q_new, test_q) < min_dist:
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
        for iter in range(10):
            q_new = self.extend(q_goal).data
            if SE2.euclid_dist(q_new, q_goal) < 1.5:
                self.goal = self.roadmap.graph[str(q_new)]
                print self.i
                return True
        return False

    @staticmethod
    def merge(T1, T2, attempts=15):
        for i in range(attempts):
            q_rand = SE2.get_random_state(greedy=True, goal=SE2(0,0,0))
            q_new1 = T1.extend(q_rand)
            q_new2 = T2.extend(q_new1.data)
            cost = SE2.distance(q_new1.data, q_new2.data)
            T1.i += 2
            if cost < 0.8:
                T1.roadmap.addNeighbor(q_new1, q_new2, cost)
                T1.roadmap.addNeighbor(q_new2, q_new1, cost)
                for node in T2.roadmap.graph.values():
                    if node.data == T1.start.data or node.data == T1.goal.data:
                        continue
                    T1.roadmap.addVertex(node)
                T1.i += T2.i
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

        if not astar:
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
                print i < len(path)-1
                if i < len(path)-1:
                    b = path[i+1]
                    print [(s.X, s.Y), (b.X, b.Y)]
                    E_a.append([(s.X, s.Y), (b.X, b.Y)])
            print 'PATH Edges', E_a
            lc1 = mc.LineCollection(E_a, color='blue')
            ax.add_collection(lc1)
            plt.scatter(Vx_a, Vy_a, color='orange')
        fig.tight_layout()
        plt.show()



if __name__ == "__main__":
    mouseBot = AckermannControl()
    q_start=SE2(-8, -6.5, 3.14/2.0)
    q_goal=SE2(9, 5.5, 3*3.14/2.0)
    samples = 100
    greedy = True
    iter_samp = 50
    '''
    for i in range(50):
        start = time()
        map = RRT(mouseBot, q_start, q_goal, greedy=greedy)
        while not map.build(samples=samples):
            samples = iter_samp

        end = time()
        path, cost = map.roadmap.AStarPath(map.start, map.goal, h=SE2.distance)
        f = open('c.txt', 'ab')
        data = str(map.i) + "\t" + str(end-start) + "\t" + str(cost) + "\n"
        f.write(data)
        f.close()
        #print map.i, end-start


        map1 = RRT(mouseBot, q_goal, q_start, greedy=greedy)

        while True:
            map.build(samples=samples)
            map1.build(samples=samples)
            map2 = RRT.merge(map, map1)
            if map2 != None:
                map = map2
                break
            samples /= 2
    '''

    #map1 = RRT(mouseBot, q_goal, q_start)
    start = time()
    map = RRT(mouseBot, q_start, q_goal, greedy=greedy)
    while not map.build(samples=samples):
        samples = iter_samp
    end = time()
    map.print_roadmap()
    print map.start, map.start.neighbors
    print map.goal, map.goal.neighbors
    #print map.goal in map.roadmap.graph
    raw_input('Start A*?')
    path, cost = map.roadmap.AStarPath(map.start, map.goal, h=SE2.distance)
    print 'cost:\t' + str(cost)
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
        sleep(.4)
