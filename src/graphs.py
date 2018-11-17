#!/usr/bin/env python
from math import fabs, sqrt, pi, cos, sin
from pyquaternion import Quaternion as Q
import sys, random
from heapq import heappush, heappop
sys.path.insert(0, '../../pqp_server/pyscript')
from pqp_ros_client import pqp_client
from Parameters import *
import tf, numpy as np

import time
random.seed(time.time())


PRECISION_DIGITS = 5

class SE2:
    def __init__(self, x, y, s, vx=0, vy=0, vs=0):
        self.X = x
        self.Y = y
        self.theta = s
        self.vx = vx
        self.vy = vy
        self.vs = vs


    def __eq__(self, other):
        return self.X == other.X and self.Y == other.Y\
         and self.theta == other.theta and self.vs == other.vs\
         and self.vy == other.vy and self.vs == other.vs

    def __str__(self):
        return str(round(self.X, PRECISION_DIGITS)) + " " +\
        str(round(self.Y, PRECISION_DIGITS)) + " " +\
        str(round(self.theta, PRECISION_DIGITS)) + " "+\
        str(round(self.vx, PRECISION_DIGITS)) + " " +\
        str(round(self.vy, PRECISION_DIGITS)) + " " +\
        str(round(self.vs, PRECISION_DIGITS))

    @staticmethod
    def distance(a, b):
        return fabs(a.theta - b.theta) + SE2.euclid_dist(a, b)

    @staticmethod
    def euclid_dist(a, b):
        return sqrt((a.X - b.X)*(a.X - b.X) + (a.Y - b.Y)*(a.Y - b.Y))

    @staticmethod
    def get_random_state(minX = -9, maxX = 10, minY = -7.5, maxY = 6.5):
        x = random.uniform(minX, maxX)
        y = random.uniform(minY, maxY)
        s = random.uniform(0, 2*pi)
        state = SE2(x, y, s)
        return state

    def unpack(self):
        list = []
        list.append(self.X)
        list.append(self.Y)
        list.append(self.theta)
        list.append(self.vx)
        list.append(self.vy)
        list.append(self.vs)
        return np.array(list)

    @staticmethod
    def repack(data):
        X = data[0]
        Y = data[1]
        theta = data[2]
        vx = data[3]
        vy = data[4]
        vs = data[5]

        return SE2(X,Y,theta,vx,vy,vs)

    @staticmethod
    def get_random_control():
        linVelMin = 0
        linVelMax = 17.8816
        steerVelMin = -244.8696
        steerVelMax = 244.8696
        # sample controls
        linVel = random.uniform(linVelMin, linVelMax)
        steerVel = random.uniform(steerVelMin, steerVelMax)
        time = random.randint(1.0,5.0)
        return [linVel, steerVel]


class SE3:
    def __init__(self,x,y,z,q):
        self.X = x
        self.Y = y
        self.Z = z
        self.q = q

    def __eq__(self, other):
        return self.X == other.X and self.Y == other.Y\
         and self.Z == other.Z and self.q == other.q

    def __str__(self):
        return str(round(self.X, PRECISION_DIGITS)) + " " +\
        str(round(self.Y, PRECISION_DIGITS)) + " " +\
        str(round(self.Z, PRECISION_DIGITS)) + " " +\
        str(round(self.q[0], PRECISION_DIGITS)) + " " +\
        str(round(self.q[1], PRECISION_DIGITS)) + " " +\
        str(round(self.q[2], PRECISION_DIGITS)) + " " +\
        str(round(self.q[3], PRECISION_DIGITS))

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
    def get_random_state(ground=False):
        minX = 0
        maxX = 8
        minY = 0
        maxY = 10
        minZ = 0.317
        maxZ = 3
        while True:
            x = random.uniform(minX, maxX)
            y = random.uniform(minY, maxY)
            z = random.uniform(minZ, maxZ)
            q = Q.random()
            if ground:
                z = 0.317
                q = tf.transformations.quaternion_from_euler(0,0,random.uniform(0,2*pi))
                q = Q(q.item(0),q.item(1),q.item(2),q.item(3))
            state = SE3(x,y,z,q)
            T, R = state.get_transition_rotation()
            if not pqp_client(T,R).result:
                # no collision
                return state
    '''
    Returns True if there is no collision
    '''
    @staticmethod
    def check_collide(a, b):
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
        or fabs(Q.sym_distance(q, b.q)) > steering_inc:
            T, R = cur.get_transition_rotation()
            if pqp_client(T,R).result:
                # Collision
                return False
            if SE3.euclid_dist(cur, b) > inc:
                x += x_inc
                y += y_inc
                z += z_inc
            if fabs(Q.sym_distance(q, b.q)) > steering_inc:
                q = Q.slerp(q, b.q, amount=steering_inc)
            cur = SE3(x, y, z, q)
        T, R = cur.get_transition_rotation()
        if pqp_client(T, R).result:
            return False
        T, R = b.get_transition_rotation()
        if pqp_client(T, R).result:
            return False
        return True


class Node:
    def __init__(self, data):
        global id_count
        self.data = data    # SE3
        self.id = str(data)
        self.neighbors = []
        self.f = sys.maxint

    def __leq__(self, other):
        return self.f < other.f

    def __str__(self):
        return str(self.data)

class Edge:
    def __init__(self, neighbor, cost):
        self.neighbor = neighbor
        self.cost = cost

    def __str__(self):
        return str(self.neighbor.data) + '\t' + str(self.cost)

class Graph:

    def __init__(self):
        self.graph = {}

    def addNeighbor(self, node, neighbor, cost):
        if node.id not in self.graph:
            self.graph[node.id] = node
        node.neighbors.append(Edge(neighbor, cost))

    def addVertex(self, node):
        if node.id not in self.graph:
            self.graph[node.id] = node

    @staticmethod
    def make_path(prev, cur):
        path = []
        while cur.id in prev:
            path.insert(0, cur.data)
            cur = prev[cur.id]
        return path

    def AStarPath(self, start, target, h=SE3.distance):
        global PRECISION_DIGITS
        prev = {}               # Previous node in optimal path from source
        dist = {}               # Unknown distance from source to v
        closed = {}
        dist[start.id] = 0      # Distance from source to source
        start.f = h(start.data, target.data)
        fringe = PriorityQueue()
        fringe.enqueue(start)
        while not fringe.is_empty():
            node = fringe.dequeue()
            if node.data == target.data:
                print dist[node.id]
                return Graph.make_path(prev, node)
            closed[node.id] = node
            for edge in node.neighbors:
                neighbor = edge.neighbor
                if neighbor.id in closed:
                    continue
                new_g = round(edge.cost + dist[node.id], PRECISION_DIGITS)
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
        return heappop(self.heap)
