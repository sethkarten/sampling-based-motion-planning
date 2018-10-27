
#include <gnn.hpp>
#include "../../include/graphs.hpp"
#include <iostream>
#include <vector>
#include "../../include/graphs.hpp"
#include "../../include/Quaternion.hpp"

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
using namespace graphs;
int main(int argc, char* argv[])
{
	Graph<Quaternion> graph = new Graph<Quaternion>();


	return 0;
}