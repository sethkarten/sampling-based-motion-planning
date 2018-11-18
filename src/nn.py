#!/usr/bin/env python
import numpy as np
import scipy as sp
from graphs import SE3,SE2
import sklearn.neighbors as sk

from time import time

def se3DistFunc(p,q):
    dist = SE3.weighted_distance(SE3.repack(p),SE3.repack(q))
    return dist
se3Dist = sk.DistanceMetric.get_metric(se3DistFunc)


def se2DistFunc(p,q):
    dist = SE2.distance(SE2.repack(p),SE2.repack(q))
    return dist

se2Dist = sk.DistanceMetric.get_metric(se2DistFunc)


class nearestNeighbor:
    def __init__(self,metric = se3DistFunc,repack = SE3.repack, se2 = False):
        if se2:
            metric = se2DistFunc
            repack = SE2.repack
        self.values = []
        self.pointCount = 0
        self.metric = metric
        self.repack = repack
        self.se2 = se2

    def addPoint(self,point):
        self.values.append(point.unpack())
        self.pointCount += 1

    def removePoint(self,point):
        point = point.unpack()
        for j in range(len(self.values)-1,-1,-1):
            isSame = True
            for k in range(0,len(self.values[j])):
                if point[k] != self.values[j][k]:
                    isSame = False
                    break
            if isSame:
                del self.values[j]

    def buildTree(self):
        arr = np.array(self.values)
        if self.se2:
            arr = np.reshape(arr, [-1, 6])
        else:
            arr = np.reshape(arr,[-1,7])
        self.nn = sk.NearestNeighbors(n_jobs=-1,algorithm='brute',leaf_size=100,metric=self.metric)
        self.nn.fit(arr)
        #self.tree = sk.BallTree(arr,metric=self.metric)

    def query_k_nearest(self,point,k):
        distances,indices = self.nn.kneighbors([point.unpack()],min(k+1,self.pointCount),return_distance=True)#self.tree.query([point.unpack()],min(k+1,self.pointCount),return_distance=True)
        distances = np.ndarray.tolist(distances)
        indices = np.ndarray.tolist(indices)
        toRemove = []
        for j in range (0,len(distances[0])):
            tmp = distances[0][j]
            if (abs(tmp) <= .0000001):
                toRemove.append(j)
        toRemove.reverse()
        for i in toRemove:
            del distances[0][i]
            del indices[0][i]
        for j in range(0,len(indices[0])):
            indices[0][j] = self.repack(self.values[indices[0][j]])
        if len(indices[0]) > k:
            del indices[0][k]
            del distances[0][k]
        return indices[0], distances[0]


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
    print "query"
    vals,dist = newg.query_k_nearest(list[0],10)
    for j in vals:
        print j
