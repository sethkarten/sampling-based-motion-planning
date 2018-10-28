#include <random>
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
unsigned long mix(unsigned long a, unsigned long b, unsigned long c)
{
	a=a-b;  a=a-c;  a=a^(c >> 13);
	b=b-c;  b=b-a;  b=b^(a << 8);
	c=c-a;  c=c-b;  c=c^(b >> 13);
	a=a-b;  a=a-c;  a=a^(c >> 12);
	b=b-c;  b=b-a;  b=b^(a << 16);
	c=c-a;  c=c-b;  c=c^(b >> 5);
	a=a-b;  a=a-c;  a=a^(c >> 3);
	b=b-c;  b=b-a;  b=b^(a << 10);
	c=c-a;  c=c-b;  c=c^(b >> 15);
	return c;
}
int main(int argc, char* argv[])
{
	std::minstd_rand0 generator;
	generator.seed(time(NULL));
	std::uniform_real_distribution<double> distribution(0.0,1.0);
	Graph<Quaternion>* graph = new Graph<Quaternion>(std::bind(&distanceQa,std::placeholders::_1,std::placeholders::_2));
	std::vector<Node<Quaternion>*> all_nodes;
	Node<Quaternion>* testN = new Node<Quaternion>(Quaternion::uniform_sample(distribution(generator),distribution(generator),distribution(generator)));
	for (int i = 0 ; i < 100; i++){
		Quaternion newQ = Quaternion::uniform_sample(distribution(generator),distribution(generator),distribution(generator));
		Node<Quaternion>* newN = new Node<Quaternion>(newQ);
		all_nodes.push_back(newN);
		graph->add_node(newN);
	}
	cout<<graph->get_nr_nodes();
	double dist = 20000000;
	cout<<"\n";
	print_node(testN);
	Node<Quaternion>* cl = (Node<Quaternion>*) graph->find_closest(testN,&dist);
	print_node(cl);
	cout<<"DIST "<<dist;
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