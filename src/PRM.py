#!/usr/bin/env python
from graphs import *
from nn import nearestNeighbor
from piano_control import PianoControl
from time import *
import threading
import math

def add_prm_node(a,b,roadmap,lock,cost):
    for n in a.neighbors:
        if str(n.neighbor) == str(b):
            return True
    if SE3.check_collide(a.data, b.data):
        if not lock is None:
            lock.acquire()
        roadmap.addNeighbor(a, b, cost)
        roadmap.addNeighbor(b, a, cost)
        if not lock is None:
            lock.release()
        return True
    return False
class CollisionThread(threading.Thread):

    def __init__(self,a,b,roadmap,lock,cost):
        threading.Thread.__init__(self)
        self.a = a
        self.b = b
        self.roadmap = roadmap
        self.lock = lock
        self.cost = cost
    def run(self):
        add_prm_node(self.a,self.b,self.roadmap,self.lock,self.cost)


class PRM:
    def __init__(self):
        self.roadmap = Graph()

    def build_connected_roadmap(self, k=3, samples=100):
        nn = nearestNeighbor()
        self.roadmap = Graph()
        for i in range(samples):
            print "getting sample "+str(i)
            new_state = SE3.get_random_state(ground=False)
            self.roadmap.addVertex(Node(new_state))
            nn.addPoint(new_state)
        print "Building tree"
        nn.buildTree()
        print "Adding Edges"

        count = 0
        for node in self.roadmap.graph.values():
            print count+1
            neighbors, distances = nn.query_k_nearest(node.data, k)
            for neighbor, cost in zip(neighbors, distances):
                string_neigh = str(neighbor)
                neighbor = self.roadmap.graph[string_neigh]
                if self.roadmap.AStarPath(node, neighbor) is None:
                    if add_prm_node(node,neighbor,self.roadmap,None,cost):
                        break
            count += 1
        return nn

    def build_dense_roadmap(self, k=3, samples=100):
        nn = nearestNeighbor()
        self.roadmap = Graph()
        for i in range(samples):
            print "getting sample "+str(i)
            new_state = SE3.get_random_state()
            self.roadmap.addVertex(Node(new_state))
            nn.addPoint(new_state)
        print "Building tree"
        nn.buildTree()
        print "Adding Edges"
        count = 0
        lock = threading.Lock()
        for node in self.roadmap.graph.values():
            runningthr = []
            print count+1
            neighbors, distances = nn.query_k_nearest(node.data, k)
            for neighbor, cost in zip(neighbors, distances):
                string_neigh = str(neighbor)
                neighbor = self.roadmap.graph[string_neigh]
                thr = CollisionThread(node,neighbor,self.roadmap,lock,cost)
                runningthr.append(thr)
                thr.start()
            count+=1
            for t in runningthr:
                t.join()
        return nn

    def build_prmstar_roadmap(self,dimensionality = 7,samples=100):
        nn = nearestNeighbor()
        self.roadmap = Graph()
        for i in range(samples):
            print "getting sample "+str(i)
            new_state = SE3.get_random_state()
            self.roadmap.addVertex(Node(new_state))
            nn.addPoint(new_state)
        print "Building tree"
        nn.buildTree()
        print "Adding Edges"

        count = 0
        lock = threading.Lock()
        for node in self.roadmap.graph.values():
            k = 0
            print count+1
            if (count != 0):
                k = int(math.ceil(math.e * (1 + (1.0/dimensionality))* math.log(count,2) ))
            tmp = time()
            neighbors, distances = nn.query_k_nearest(node.data, k)
            print "nn check took ",time()-tmp,"(found ",len(neighbors)," neighbors)"
            tmp = time()

            runningthr = []
            for neighbor, cost in zip(neighbors, distances):
                string_neigh = str(neighbor)
                neighbor = self.roadmap.graph[string_neigh]

                thr = CollisionThread(node,neighbor,self.roadmap,lock,cost)
                runningthr.append(thr)
                thr.start()
            count += 1
            for t in runningthr:
                t.join()
            print "col check took",time()-tmp
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
    k = 5
    numsamples = 100
    maps = [PRM(),PRM(),PRM()]
    nns = []
    nns.append(maps[0].build_connected_roadmap(k,numsamples))
    nns.append(maps[1].build_dense_roadmap(k,numsamples))
    nns.append(maps[2].build_prmstar_roadmap(dimensionality=6,samples=numsamples))
    for map,nn in zip(maps,nns):
        print "Building roadmap"
        global TOTAL_TIME_STR
        print getGlobalTimeStr(), "TOTAL TIME STR"
        while True:
            text = raw_input('Get new path? Y/N')
            if text == 'N':
                break
            start = Node(SE3.get_random_state(ground=True))
            goal = Node(SE3.get_random_state(ground=True))
            map.add_points(start, goal, nn, k)

            print "Start: "+str(start), "Goal: "+str(goal)

            path = map.roadmap.AStarPath(start, goal)
            if path == None:
                continue
            path_s = []
            for s in path:
                path_s.append(str(s))
            print path_s

            raw_input('Start A*?')

            rocketPiano = PianoControl()
            rocketPiano.set_position(start.data)
            rocketPiano.set_steering_angle(start.data.q)
            print start.data
            for state in path:
                sleep(.01)
                print state
                rocketPiano.interpolate(state)

