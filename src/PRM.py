#!/usr/bin/env python
from graphs import *
from nn import nearestNeighbor
from piano_control import PianoControl
from time import sleep
import math
class PRM:
    def __init__(self):
        self.roadmap = Graph()

    def build_connected_roadmap(self, k=3, samples=100):
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
        for node in self.roadmap.graph.values():
            print count+1
            neighbors, distances = nn.query_k_nearest(node.data, k)
            for neighbor, cost in zip(neighbors, distances):
                string_neigh = str(neighbor)
                neighbor = self.roadmap.graph[string_neigh]
                if self.roadmap.AStarPath(node, neighbor) is None:
                    if SE3.check_collide(node.data, neighbor.data):
                        self.roadmap.addNeighbor(node, neighbor, cost)
                        self.roadmap.addNeighbor(neighbor, node, cost)
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
        for node in self.roadmap.graph.values():
            print count+1
            neighbors, distances = nn.query_k_nearest(node.data, k)
            for neighbor, cost in zip(neighbors, distances):
                string_neigh = str(neighbor)
                neighbor = self.roadmap.graph[string_neigh]
                if SE3.check_collide(node.data, neighbor.data):
                    self.roadmap.addNeighbor(node, neighbor, cost)
                    self.roadmap.addNeighbor(neighbor, node, cost)
            count+=1
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
        for node in self.roadmap.graph.values():
            k = 0
            print count+1
            if (count != 0):
                k = int(math.ceil(math.e * (1 + (1.0/dimensionality))* math.log(count,2) ))
            neighbors, distances = nn.query_k_nearest(node.data, k)
            for neighbor, cost in zip(neighbors, distances):
                string_neigh = str(neighbor)
                neighbor = self.roadmap.graph[string_neigh]
                if SE3.check_collide(node.data, neighbor.data):
                    self.roadmap.addNeighbor(node, neighbor, cost)
                    self.roadmap.addNeighbor(neighbor, node, cost)
            count += 1
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
    numsamples = 500
    map = PRM()
    print "Building roadmap"
    nn = map.build_prmstar_roadmap(k, samples = numsamples)

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
            sleep(2)
            print state
            rocketPiano.interpolate(state)
