//
// Created by sg on 10/26/18.
//

#ifndef SAMPLING_GRAPH_H
#define SAMPLING_GRAPH_H

#include "gnn.hpp"
using namespace gnn;
namespace graphs {
    template <class T>
    class Node : public proximity_node_t{
    public:
        Node(T& data);
        T& storedData;
        double g,h;
        std::vector<std::pair<Node<T>&,double>> neighbors;
        bool operator <(const Node &n);
        void addNeighbor(Node<T>& n,double cost);
    };
    template <class T>
    class Graph : public graph_nearest_neighbors_t{
    public:
        std::unordered_set<Node<T>&> vertices;
        Graph();
        
    };
};

#endif //SAMPLING_GRAPH_H
