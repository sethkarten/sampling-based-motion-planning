from math import fabs, sqrt
from pyquaternion import Quaternion as Q
import sys, random, numpy as np
from heapq import heappush, heappop
sys.path.insert(0, '../../pqp_server/pyscript')
from pqp_ros_client import pqp_client

id_count = 0
PRECISION_DIGITS = 5

class SE3:
    def __init__(self,x,y,z,q):
        self.X = x
        self.Y = y
        self.Z = z
        self.q = q


    def unpack(self):
        list = []
        list.append(self.X)
        list.append(self.Y)
        list.append(self.Z)
        tmp = self.q.elements
        for j in tmp:
            list.append(j)
        return np.array(list)
    @staticmethod
    def repack(data):
        x = data[0]
        y = data[1]
        z = data[2]
        q = Q(data[3],data[4],data[5],data[6])
        return SE3(x,y,z,Q(q))

    def __eq__(self, other):
        return self.X == other.X and self.Y == other.Y\
         and self.Z == other.Z and self.q == other.q

    @staticmethod
    def distance(a, b):
        return Q.sym_distance(a.q, b.q) + SE3.euclid_dist(a, b)

    @staticmethod
    def euclid_dist(a, b):
        return sqrt((a.X - b.X)*(a.X - b.X) + (a.Y - b.Y)*(a.Y - b.Y) +  (a.Z - b.Z)*(a.Z - b.Z))

    def get_transition_rotation(self):
        T = [self.X, self.Y, self.Z]
        R = self.q.rotation_matrix.reshape(9).tolist()
        return T, R

    @staticmethod
    def get_random_state():
        minX = -1
        maxX = 1
        minY = -1
        maxY = 1
        minZ = -1
        maxZ = 1
        while True:
            x = random.uniform(minX, maxX)
            y = random.uniform(minY, maxY)
            z = random.uniform(minZ, maxZ)
            q = Q.random()
            state = SE3(x,y,z,q)
            T, R = state.get_transition_rotation()
            if pqp_client(T, R):
                # no collision
                return state
    '''
    Returns True if there is a collision
    '''
    @staticmethod
    def check_collide(self, other):
        x_res = y_res = z_res = 0.1
        if self.X > other.X:
            x_res *= -1
        if self.Y > other.Y:
            x_res *= -1
        if self.Z > other.Z:
            x_res *= -1
        x = self.X
        y = self.Y
        z = self.Z
        q = self.q
        while fabs(x - other.X) > 0.1\
        and fabs(y - other.Y) > 0.1\
        and fabs(z - other.Z) > 0.1\
        and fabs(Q.sym_distance(q, other.q)) > 0.1:
            T, R = self.get_transition_rotation()
            if pqp_client(T, R):
                # Collision
                return True
            x += x_res
            y += y_res
            z += z_res
            q = Q.slerp(q, other.q, amount=0.1)
        return False


class Node:
    def __init__(self, data):
        global id_count
        self.data = data    # SE3
        self.id = id_count
        self.neighbors = []
        id_count += 1
        self.f = sys.maxint

    def __leq__(self, other):
        return self.f < other.f

class Edge:
    def __init__(self, neighbor, cost):
        self.neighbor = neighbor
        self.cost = cost

class Graph:

    def __init__(self):
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

    def make_path(prev, cur):
        path = []
        while cur.id in prev:
            path.insert(0, cur.value)
            cur = prev[cur.id]
        return path

    def AStarPath(start, target, d=SE3.distance, h=SE3.distance):
        global PRECISION_DIGITS
        prev = {}               # Previous node in optimal path from source
        dist = {}               # Unknown distance from source to v
        closed = {}
        dist[start.id] = 0      # Distance from source to source
        start.f = h(start, target)
        fringe = PriorityQueue()
        fringe.enqueue(start)
        while not fringe.empty():
            node = fringe.dequeue()
            if node.data == target.data:
                print dist[node.id]
                return make_path(prev, node)
            closed[node.id] = node
            for neighbor in node.neighbors:
                if neighbor.id in closed:
                    continue
                new_g = round(d(node.data, neighbor.data) + dist[node.id], PRECISION_DIGITS)
                if neighbor.id in dist:
                    if new_g >= dist[neighbor.id]:
                        continue
                dist[neighbor.id] = new_g
                prev[neighbor.id] = node
                neighbor.f = new_g + h(neighbor.data, target.data)
                fringe.enqueue(neighbor)

class PriorityQueue:
    def __init__(self):
        self.heap = []

    def enqueue(self, node):
        heappush(self.heap, node)

    def is_empty(self):
        return len(self.heap) == 0

    def dequeue(self):
        return heapop(self.heap)
