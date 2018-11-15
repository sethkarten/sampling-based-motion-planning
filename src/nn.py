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
        return self.tree.query(point,k,return_distance=False)

if __name__ == "__main__":
    newg = nearestNeighbor()
    list = []
    for i in range(0,100):
        print i
        tmp = SE3.get_random_state()
        print tmp.q
        list.append(tmp)
        newg.addPoint(tmp)

    newg.buildTree()
    print newg.values[0]
    print newg.values[1]
    print newg.query_k_nearest([newg.values[0]],2)