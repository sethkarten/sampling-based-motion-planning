from math import fabs
from pyquaternion import Quaternion as Q
import sys
sys.path.insert(0, '../../pqp_server/pyscript')
from pqp_ros_client import pqp_client
id_count = 0

class SE3:
    def __init__(x,y,z,q):
        self.X = x
        self.Y = y
        self.Z = z
        self.q = q

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
        and fabs(Z - other.Z) > 0.1\
        and fabs(Q.sym_distance(q, other.q)) > 0.1:
            T = [x,y,z]
            R = q.rotation_matrix.reshape(9).tolist()
            if pqp_client(T, R):
                # Collision
                return True
            x += x_res
            y += y_res
            z += z_res
            q = Quaternion.slerp(q, other.q, amount=0.1)
        return False


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
