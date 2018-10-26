
#include "gnn.hpp"

#include <iostream>
#include <vector>

class state_node_t : public gnn::proximity_node_t
{
public:
	state_node_t(int dim) : proximity_node_t()
	{
		state = new double[dim];
		d = dim;
	}
	virtual ~state_node_t()
	{
		delete state;
	}
	double* state;
	int d;
};

double distance(gnn::proximity_node_t* in_s, gnn::proximity_node_t* in_t)
{
	auto s = (state_node_t*)in_s;
	auto t = (state_node_t*)in_t;
	double sum=0;
	for(int i=0;i<s->d;++i)
		sum += pow(s->state[i]-t->state[i],2.0);
	return sqrt(sum);
}


double uniform_random()
{
    double val = rand()*1.0 / RAND_MAX;
    return val;
}

void print_node(gnn::proximity_node_t* node)
{
	auto n = (state_node_t*)node;
	for(int i=0;i<n->d-1;++i)
	{
		std::cout<<n->state[i]<<", ";
	}
	std::cout<<n->state[n->d-1]<<std::endl;
}

int main(int argc, char* argv[])
{
	int dimension = 2;
	gnn::graph_nearest_neighbors_t* g;
	g = new gnn::graph_nearest_neighbors_t(std::bind(&distance,std::placeholders::_1,std::placeholders::_2));

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

	return 0;
}