#!/usr/bin/env python
from math import fabs, sqrt, pi
from pyquaternion import Quaternion as Q
import sys, random
from heapq import heappush, heappop
sys.path.insert(0, '../../pqp_server/pyscript')
from pqp_ros_client import pqp_client
import tf, numpy as np

id_count = 0
PRECISION_DIGITS = 5

class SE2:
    def __init__(self, x, y, theta):
        self.X = x
        self.Y = y
        self.theta = theta

    def __eq__(self, other):
        return self.X == other.X and self.Y == other.Y\
         and self.theta == other.theta

    @staticmethod
    def distance(a, b):
        return fabs(a.theta - other.theta) + euclid_dist(a, b)

    @staticmethod
    def euclid_dist(a, b):
        return sqrt((a.X - b.X)*(a.X - b.X) + (a.Y - b.Y)*(a.Y - b.Y))

    @staticmethod
    def get_random_state():
        minX = -9
        maxX = 10
        minY = -7.5
        maxY = 6.5
        while True:
            x = random.uniform(minX, maxX)
            y = random.uniform(minY, maxY)
            theta = random.uniform(0, 2*pi)
            state = SE2(x, y, theta)
            T = [x,y,0]
            rot = tf.transformations.euler_matrix(0,0,theta)
            R = [rot.item(0,0),rot.item(0,1),rot.item(0,2),\
            rot.item(1,0),rot.item(1,1),rot.item(1,2),\
            rot.item(2,0),rot.item(2,1),rot.item(2,2)]
            if pqp_client(T, R):
                # no collision
                return state


class SE3:
    def __init__(self,x,y,z,q):
        self.X = x
        self.Y = y
        self.Z = z
        self.q = q

    def __eq__(self, other):
        return self.X == other.X and self.Y == other.Y\
         and self.Z == other.Z and self.q == other.q

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
        minX = -10
        maxX = 10
        minY = -10
        maxY = 10
        minZ = 0
        maxZ = 10
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
    def check_collide(a, b):
        inc = 0.1
        x_inc = b.X - a.X
        y_inc = b.Y - a.Y
        z_inc = b.Z - a.Z
        mag = sqrt(x_inc*x_inc + y_inc*y_inc + z_inc*z_inc)
        x_inc = x_inc / mag * inc
        y_inc = y_inc / mag * inc
        z_inc = z_inc / mag * inc
        x = a.X
        y = a.Y
        z = a.Z
        q = a.q
        cur = a
        while SE3.euclid_dist(cur, b) > inc\
        or fabs(Q.sym_distance(q, b.q)) > 0.02:
            T, R = cur.get_transition_rotation()
            if pqp_client(T, R):
                # Collision
                return True
            if SE3.euclid_dist(cur, b) > inc:
                x += x_inc
                y += y_inc
                z += z_inc
            if fabs(Q.sym_distance(q, b.q)) > 0.02:
                q = Q.slerp(q, b.q, amount=0.02)
            cur = SE3(x, y, z, w)
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
