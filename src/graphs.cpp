//
// Created by sg on 10/26/18.
//

#include "../include/graphs.hpp"
#include "../include/graphs.hpp"

using namespace graphs;
template <class T>
void Node<T>::addNeighbor(graphs::Node<T> &n, double cost) {
    this->neighbors.push_back(std::make_pair(n,cost));
}


template <class T>
Node<T>::Node(T& data) {
    this->storedData = data;
    this.g = 0;
    this.h = 0;
}

template <class T>
Graph<T>::Graph(){
    this->vertices.clear();
}
template <class T>
void Graph<T>::addVertex(T &data) {
    Node<T>& n = new Node<T>(data);
    this->vertices.insert(n);
}
template <class T>
bool Node<T>::operator<(const Node<T> &n) {
    return this.g + this.h < n.g + n.h;
}

