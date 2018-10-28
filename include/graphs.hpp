//
// Created by sg on 10/26/18.
//

#ifndef SAMPLING_GRAPH_H
#define SAMPLING_GRAPH_H

#include <gnn.hpp>

using namespace gnn;
namespace graphs {
    template<class T>
    class Node : public proximity_node_t {
    public:
        T storedData;
        double g, h;
        Node(T data):proximity_node_t(){
            this->storedData = T(data);
            this->g = 0;
            this->h = 0;
        };


        bool operator<(const Node &n) const;
        bool operator=(const Node &n) const;
        void addNeighbor(Node<T> &n, double cost);
    };
}
namespace std{
    template<class T>
    class hash<graphs::Node<T>>{
    public:
        size_t operator()(const graphs::Node<T>& n) const;
    };
}
namespace graphs{
    template <class T>
    class Graph : public graph_nearest_neighbors_t{
    public:
        Graph(std::function<double(proximity_node_t*, proximity_node_t*)>fun):graph_nearest_neighbors_t(fun){

        }
        void addVertex(T& data);

    };
};
#endif //SAMPLING_GRAPH_H
