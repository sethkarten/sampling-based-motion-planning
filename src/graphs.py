from Quaternion import *

id_count = 0

class Node:
    def __init__(data):
        global id_count
        self.data = data    # SE3
        self.g = 0
        self.h = 0
        self.id = id_count
        self.neighbors = []
        id_count += 1

    def __leq__(self, other):
        return self.g + self.h < other.g + other.h

class Edge:
    def __init__(neighbor, cost):
        self.neighbor = neighbor
        self.cost = cost

class Graph:

    def __init__():
        self.graph = {}

    def addNeighbor(self, node, neighbor, cost):
        if self.graph[node.id_count] is None:
            self.graph[node.id_count] = node
        node.neighbors.append(Edge(neighbor, cost))

    def addVertex(self, node):
        if self.graph[node.id_count] is None:
            self.graph[node.id_count] = node

    def buildNeighbor(self):
        return
