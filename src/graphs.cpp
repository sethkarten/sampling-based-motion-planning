//
// Created by sg on 10/26/18.
//

#include "../include/graphs.hpp"

using namespace graphs;
namespace std{
    template<class T>
    class hash<Node<T>>{
    public:
        size_t operator()(const Node<T>& n) const{
            return std::hash<T>()(n.storedData);
        };
    };
}
template <class T>
void Node<T>::addNeighbor(graphs::Node<T> &n, double cost) {
    this->neighbors.push_back(std::make_pair(n,cost));
}


template <class T>
void Graph<T>::addVertex(T &data) {
    Node<T>& n = *(new Node<T>(data));

    this->vertices.insert(n);
    this->add_node(n);
}
template <class T>
bool Node<T>::operator<(const Node<T> &n) {
    return this.g + this.h < n.g + n.h;
}
template <class T>
bool Node<T>::operator=(const Node<T> &n) {
    return n.storedData == this->storedData;
}
