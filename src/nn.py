#!/usr/bin/env python
import numpy as np
import scipy as sp
from graphs import SE3
import sklearn.neighbors as sk


def se3DistFunc(p,q):
    dist = SE3.distance(SE3.repack(p),SE3.repack(q))
    return dist
se3Dist = sk.DistanceMetric.get_metric(se3DistFunc)

class nearestNeighbor:
    def __init__(self):
        self.values = []
        self.pointCount = 0

    def addPoint(self,point):
        self.values.append(point.unpack())
        self.pointCount += 1

    def buildTree(self):
        arr = np.array(self.values)
        arr = np.reshape(arr,[-1,7])
        self.tree = sk.BallTree(arr,metric=se3Dist)

    def query_k_nearest(self,point,k):
        distances,indices = self.tree.query([point.unpack()],min(k+1,self.pointCount),return_distance=True)
        distances = np.ndarray.tolist(distances)
        indices = np.ndarray.tolist(indices)
        toRemove = []
        for j in range (0,len(distances[0])):
            tmp = distances[0][j]
            if (abs(tmp) <= .0000001):
                toRemove.append(j)
        toRemove.reverse()
        for i in toRemove:
            print "deleting",i
            del distances[0][i]
            del indices[0][i]
        for j in range(0,len(indices)):
            indices[0][j] = SE3.repack(self.values[indices[0][j]])
        print "done query"
        return indices, distances

if __name__ == "__main__":
    newg = nearestNeighbor()
    list = []
    for i in range(0,100):
        print i
        print "getting rand"
        tmp = SE3.get_random_state()
        print tmp
        list.append(tmp)
        newg.addPoint(tmp)

    newg.buildTree()
    print newg.values[0]
    print newg.values[1]
    print "query"
    print newg.query_k_nearest(list[0],2)
