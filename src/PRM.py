#!/usr/bin/env python
from graphs import *
from nn import nearestNeighbor
from piano_control import PianoControl
from time import *
import threading
import math
import matplotlib.pyplot as plt
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

    def addToRoadmap(self,nn,point):
        nn.addPoint(point)
        self.roadmap.addVertex(Node(point))

    def build_connected_roadmap(self, k=3, samples=100,points = None):
        nn = nearestNeighbor()
        self.roadmap = Graph()
        if points is None:
            for i in range(samples):
                print "getting sample "+str(i)
                new_state = SE3.get_random_state(ground=False)
                self.roadmap.addVertex(Node(new_state))
                nn.addPoint(new_state)
        else:
            for p in points:
                self.addToRoadmap(nn,p)
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
                if self.map.roadmap.AStarPath(node, neighbor):
                    if add_prm_node(node,neighbor,self.roadmap,None,cost):
                        break
            count += 1
        return nn

    def build_dense_roadmap(self, k=3, samples=100,points = None):
        nn = nearestNeighbor()
        self.roadmap = Graph()
        if points is None:
            for i in range(samples):
                print "getting sample "+str(i)
                new_state = SE3.get_random_state(ground=False)
                self.roadmap.addVertex(Node(new_state))
                nn.addPoint(new_state)
        else:
            for p in points:
                self.addToRoadmap(nn,p)
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

    def build_prmstar_roadmap(self,dimensionality = 7,samples=100,points = None):
        nn = nearestNeighbor()
        self.roadmap = Graph()
        if points is None:
            for i in range(samples):
                print "getting sample "+str(i)
                new_state = SE3.get_random_state(ground=False)
                self.roadmap.addVertex(Node(new_state))
                nn.addPoint(new_state)
        else:
            for p in points:
                self.addToRoadmap(nn,p)
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
            #print "nn check took ",time()-tmp,"(found ",len(neighbors)," neighbors)"
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
            #print "col check took",time()-tmp
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

    def remove_points(self,node1,node2,nn):
        for node in [node1, node2]:
            point = node.data
            self.roadmap.removeVertex(node)
            nn.removePoint(point)
            nn.buildTree()

class DriverThread(threading.Thread):

    def __init__(self,map,nn,buildtime,color,lock,xData,yData,cData,k,startGoalSamples = None):
        threading.Thread.__init__(self)
        self.map = map
        self.nn = nn
        self.buildtime = buildtime
        self.color = color
        self.lock = lock
        self.xData = xData
        self.yData = yData
        self.cData = cData
        self.k = k
        self.startGoalSamples = startGoalSamples
    def run(self):
        count = 0
        while count < 50:
            # text = raw_input('Get new path? Y/N')
            # if text == 'N':
            #    break
            print "Sample ", count
            start = Node(SE3.get_random_state(ground=True))
            goal = Node(SE3.get_random_state(ground=True))
            self.map.add_points(start, goal, self.nn, self.k)
            # print "Start: "+str(start), "Goal: "+str(goal)
            tmp = time()
            path = self.map.roadmap.AStarPath(start, goal)
            dt = time() - tmp
            if path == None:
                continue
            curr = None
            dist = 0
            # print len(path)
            for p in path:
                if not curr is None:
                    # print curr,p
                    dist = dist + SE3.distance(curr, p)
                curr = p
            dist += SE3.distance(curr, start.data)
            self.lock.acquire()
            self.xData.append(dt)#self.buildtime)
            self.yData.append(SE3.distance(start.data, goal.data) / dist)
            self.cData.append(self.color)
            self.lock.release()
            count += 1
            self.map.remove_points(start,goal,self.nn)

if __name__ == '__main__':
    for numsamples in [25,500]:
        k = 5
        maps = [PRM(),PRM(),PRM()]
        nns = []
        buildTimes = []

        randomSamples = []
        for i in range(0,numsamples):
            randomSamples.append(SE3.get_random_state(ground=False))
        tmp = time()
        nns.append(maps[0].build_connected_roadmap(k,numsamples,points=randomSamples))
        buildTimes.append(time()-tmp)
        tmp = time()
        nns.append(maps[1].build_dense_roadmap(k,numsamples,points=randomSamples))
        buildTimes.append(time() - tmp)
        tmp = time()
        nns.append(maps[2].build_prmstar_roadmap(dimensionality=6,samples=numsamples,points=randomSamples))
        buildTimes.append(time() - tmp)

        colors = ['r','g','b']

        lock = threading.Lock()
        xData = []
        yData = []
        cData = []
        runningthr = []

        startGoalSamples = []
        startGoalCount = 0


        for map,nn,buildtime,color in zip(maps,nns,buildTimes,colors):
                thr = DriverThread(map,nn,buildtime,color,lock,xData,yData,cData,k=k,startGoalSamples = startGoalSamples)
                runningthr.append(thr)
                thr.start()
                """
                raw_input('Start A*?')
    
                rocketPiano = PianoControl()
                rocketPiano.set_position(start.data)
                rocketPiano.set_steering_angle(start.data.q)
                print start.data
                for state in path:
                    sleep(.01)
                    print state
                    rocketPiano.interpolate(state)
                """
        for t in runningthr:
            t.join()
        plt.scatter(xData,yData,c=cData)
        plt.show()