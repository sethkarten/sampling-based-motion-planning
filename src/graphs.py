#!/usr/bin/env python
from math import fabs, sqrt, pi, cos, sin
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
        maxY = 6.
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
            if not pqp_client(T,R).result:
                # no collision
                return state

    @staticmethod
    def get_random_control(state, vel):
        linAccMin = -10.0
        linAccMax = 10.0
        steerAccMin = -5.0
        steerAccMax = 5.0
        # sample controls
        collision = False
        #while True:
        linAcc = random.uniform(linAccMin, linAccMax)
        steerAcc = random.uniform(steerAccMin, steerAccMax)
        time = random.random()
        inc = 0.05
        t0 = 0
        vx, vy, vs = vel[0], vel[1], vel[2]
        x, y, s  = state.X, state.Y, state.theta
        while t0 < time:
            x += (vx*inc + linAcc*inc*inc) * cos(s) * cos(steerAcc)
            vx += linAcc * inc * cos(s) * cos(steerAcc)
            y += (vy*inc + linAcc*inc*inc) * sin(s) * cos(steerAcc)
            vy += linAcc * inc * sin(s) * cos(steerAcc)
            s += vs*inc + steerAcc*inc*inc * sin(steerAcc)
            vs += steerAcc * inc * sin(steerAcc)
            t0 += inc
        return (SE2(x,y,s), [vx,vy,vs])
        '''
                T = [x,y,0]
                rot = tf.transformations.euler_matrix(0,0,s)
                R = [rot.item(0,0),rot.item(0,1),rot.item(0,2),\
                rot.item(1,0),rot.item(1,1),rot.item(1,2),\
                rot.item(2,0),rot.item(2,1),rot.item(2,2)]
                collision = pqp_client(T,R).result

                if collision:
                    # Collision
                    break

            if not collision:
        '''

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
        return str(self.X) + " " + str(self.Y) + " " + str(self.Z) + " " + str(self.q)

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

    def __str__(self):
        quaternstring = self.q.__str__()
        totalstr = str(self.X) + " " + str(self.Y) + " " + str(self.Z) + " " + quaternstring
        return totalstr
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
        minX = 0
        maxX = 10
        minY = 0
        maxY = 10
        minZ = 0
        maxZ = 3
        while True:
            x = random.uniform(minX, maxX)
            y = random.uniform(minY, maxY)
            z = random.uniform(minZ, maxZ)
            q = Q.random()
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
        inc = 0.1
        x_inc = b.X - a.X
        y_inc = b.Y - a.Y
        z_inc = b.Z - a.Z
        mag = sqrt(x_inc*x_inc + y_inc*y_inc + z_inc*z_inc)
        x_inc = x_inc / mag * inc
        y_inc = y_inc / mag * inc
        z_inc = z_inc / mag * inc
        steps = mag / ((x_inc+y_inc+z_inc)/3)
        steering_inc = 0.03
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
        return True


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
        if node.id not in self.graph:
            self.graph[node.id] = node
        node.neighbors.append(Edge(neighbor, cost))

    def addVertex(self, node):
        if node.id not in self.graph:
            self.graph[node.id] = node

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
