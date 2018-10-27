
#include "../nearest_neighbors/include/gnn.hpp"
#include "../include/graphs.hpp"
#include <iostream>
#include <vector>
#include "../include/graphs.hpp"
#include "../include/Quaternion.hpp"

using namespace graphs;
double distanceQa(gnn::proximity_node_t* in_s, gnn::proximity_node_t* in_t)
{
	auto s = (Node<Quaternion>*)in_s;
	auto t = (Node<Quaternion>*)in_t;
	return s->storedData.distance(t->storedData);
}


double uniform_random()
{
    double val = rand()*1.0 / RAND_MAX;
    return val;
}

void print_node(gnn::proximity_node_t* node)
{
	auto n = (Node<Quaternion>*)node;
}
using namespace graphs;
int main(int argc, char* argv[])
{
	Graph<Quaternion>* graph = new Graph<Quaternion>(std::bind(&distanceQa,std::placeholders::_1,std::placeholders::_2));
	/*
	std::vector<state_node_t*> all_nodes;
	state_node_t* query_node = new state_node_t(dimension);
	for(int i=0;i<10000;++i)
	{
		auto new_node = new state_node_t(dimension);
		for(int j=0;j<dimension;++j)
		{
			//create a random point between -100,100
			new_node->state[j] = uniform_random()*200-100;
		}
		g->add_node(new_node);
		all_nodes.push_back(new_node);
	}
	for(int j=0;j<dimension;++j)
	{
		//create a random point between -100,100
		query_node->state[j] = uniform_random()*200-100;
	}

	double dist;
	std::cout<<"Query Node: ";
	print_node(query_node);
	std::cout<<"Closest Node: ";
	print_node(g->find_closest(query_node,&dist));

	for(int i=0;i<10000;++i)
	{
		g->remove_node(all_nodes[i]);
	}
	delete query_node;
	*/
	return 0;
}