#!/usr/bin/env python
from matplotlib import collections as mc, pyplot as plt
import pylab as pl
import rospy, numpy as np
import sys

if __name__ == '__main__':
    matlab = True
    ROS = False

    if ROS:
        rocketPiano = botClient()


    disgraph = map_prep.discretise(resolution=.1) # Change resolution here

    if matlab:
        '''
        for obs in map_prep.obstacles:
            for j in obs.vertices:
                print str(j.x)+' , '+str(j.y), '\t\t'
            print ""
        '''
        fig = plt.figure()
        ax = fig.add_subplot(111)
        grid = plt.grid(b=True)
        cvals = []
        xvals = []
        yvals = []
        for j in disgraph.discretizedmap:
            for k in disgraph.discretizedmap[j]:
                value = disgraph.discretizedmap[j][k]
                xvals.append(float(j))
                yvals.append(float(k))
                if value == -1:
                    cvals.append("black")
                elif value == 0:
                    cvals.append("blue")
                elif value == 1:
                    cvals.append("red")
                else:
                    cvals.append("yellow")

    graph = disgraph.build_graph()


    if matlab:
        xs=-4
        ys=3
        xg=3
        yg=-5
        startvert = graph.get_vertex_by_name(disgraph.stringify_coord(MapUtilities.Coordinate(xs, ys)))
        endvert = graph.get_vertex_by_name(disgraph.stringify_coord(MapUtilities.Coordinate(xg, yg)))
    if ROS:
        rate = rospy.Rate(10)   #10 hz
        tuttleTheBot.get_position()
        rate.sleep()
        tuttleTheBot.get_position()
        rate.sleep()
        startvert = graph.get_vertex_by_name(disgraph.stringify_coord(MapUtilities.Coordinate(tuttleTheBot.cur_x, tuttleTheBot.cur_y)))
        goal_coords = np.array(rospy.get_param("/goal_position"))
        endvert = graph.get_vertex_by_name(disgraph.stringify_coord(MapUtilities.Coordinate(goal_coords[0], goal_coords[1])))
        print startvert.data['coord'], endvert.data['coord']

    path = Pathfinding.AStarPath(graph,startvert,endvert)

    xpath = []
    ypath = []
    if matlab:
        cpath = []
    for i in range(len(map_prep.obstacles)):
        map_prep.obstacles[i] = raw_obs[i].minkowski_sum(bot_radius=.2)
    if len(path) == 0:
        'No path found'
        sys.exit(1)
    for v in path:
        v = graph.get_vertex_by_name(v)
        xpos = v.data['x']
        ypos = v.data['y']
        xpath.append(xpos)
        ypath.append(ypos)
        if matlab:
            cpath.append("yellow")
    if matlab:
        plt.scatter(xvals+xpath,yvals+ypath,c=cvals+cpath)
        plt.grid()
        #pl.savefig(a+str(h)+'.png', bbox_inches = 'tight')
        fig, ax = pl.subplots()
        for obstacle in raw_obs:
            obs = obstacle.get_segments() + raw_boundary.get_segments()
            lc = mc.LineCollection(obs, color='blue', linewidths=2)
            ax.add_collection(lc)
        for obstacle in map_prep.obstacles:
            obs = obstacle.get_segments() + map_prep.boundary.get_segments()
            lc = mc.LineCollection(obs, color='red', linewidths=2)
            ax.add_collection(lc)
        lines = []
        for i in range(len(path)-1):
            cur = graph.get_vertex_by_name(path[i])
            next = graph.get_vertex_by_name(path[i+1])
            lines.append([(cur.data['x'] ,cur.data['y']), (next.data['x'], next.data['y'])])
        lc2 = mc.LineCollection(lines, color='orange', linewidths=0.5)
        ax.add_collection(lc2)
        plt.scatter(np.array(startvert.data['x']),np.array(startvert.data['y']), color='red', linewidths=3)
        plt.scatter(np.array(endvert.data['x']), np.array(endvert.data['y']), color='green', linewidths=3)
        fig.tight_layout()
        #plt.savefig(b+str(h)+'.png', bboxinches = 'tight')
        plt.show()


    if ROS:
        ''' ROS communication to move along A* path'''
        #rate = rospy.Rate(10)   #10 hz
        print xpath, ypath
        rate = rospy.Rate(10)   #10 hz
        for x,y in zip(xpath, ypath):
            print x,y
            tuttleTheBot.update_position(x,y)
            tuttleTheBot.get_position()
            rate.sleep()
            #while abs(tuttleTheBot.cur_x - x) > 0.1 and abs(tuttleTheBot.cur_y - y) > 0.1:
                #tuttleTheBot.get_position()
        print "Simulation terminated"
