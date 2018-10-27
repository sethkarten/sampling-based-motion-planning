//
// Created by sg on 10/26/18.
//

#include "../include/graphs.hpp"
using namespace graphs;
template <class T>
void Node<T>::addNeighbor(graphs::Node<T> &n, double cost) {
    this.g = 0;
    this.h = 0;
    this->neighbors.push_back(std::make_pair(n,cost));
}
template <class T>
Node<T>::Node(T &data) {
    this->storedData = data;
}
template <class T>
bool Node<T>::operator<(const Node<T> &n) {
    return this.g + this.h < n.g + n.h;
}

