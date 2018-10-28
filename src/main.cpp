
#include "../nearest_neighbors/include/gnn.hpp"
#include "../include/graphs.hpp"
#include <iostream>
#include <vector>
#include "../include/graphs.hpp"
#include "../include/Quaternion.hpp"

using namespace std;
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
void print_quat(Quaternion q){
	cout<<q.W<<" "<<q.X<<" "<<q.Y<<" "<<q.Z<<"\n";
}
void print_node(Node<Quaternion>* n)
{
	cout<<n->storedData.W<<" "<<n->storedData.X<<" "<<n->storedData.Y<<" "<<n->storedData.Z<<"\n";
}
int main(int argc, char* argv[])
{
	Graph<Quaternion>* graph = new Graph<Quaternion>(std::bind(&distanceQa,std::placeholders::_1,std::placeholders::_2));
	std::vector<Node<Quaternion>*> all_nodes;
	for (int i = 0 ; i < 2; i++){
		Quaternion newQ = Quaternion::uniform_sample(100);
		Node<Quaternion>* newN = new Node<Quaternion>(newQ);
		print_quat(newQ);
		print_node(newN);
		all_nodes.push_back(newN);
		graph->add_node(newN);
		print_node(newN);
	}
	cout<<graph->get_nr_nodes();
	double dist = 20000000;
	cout<<"\n";
	print_node(all_nodes[1]);
	Node<Quaternion>* cl = (Node<Quaternion>*) graph->find_closest(all_nodes[0],&dist);
	print_node(cl);
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