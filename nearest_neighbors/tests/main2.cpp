#include "gnn.hpp"

#include <iostream>
#include <vector>
#include <stdlib.h>
#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>
#include <chrono>

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

double uniform_random();
double distance(gnn::proximity_node_t*, gnn::proximity_node_t*);
void print_node(gnn::proximity_node_t*);


gnn::graph_nearest_neighbors_t* buildGraph(std::string, int);
void serializeGraph(gnn::graph_nearest_neighbors_t*, std::string);
gnn::graph_nearest_neighbors_t* deserializeGraph(std::string nodesFile, std::string edgesFile, int dim);

int main(int argc, char* argv[])
{
	int dimension = 9;
	
	double dist1;
	double dist2;

	int nr_queries = 5;

	//build NN graph given a bunch of sample points
	gnn::graph_nearest_neighbors_t* g = buildGraph("map.roadmap", dimension);

	//stores all points with their neighbors in edges.txt file
	serializeGraph(g, "edges.txt");

	//given sample points and their neighbors build GNN
	gnn::graph_nearest_neighbors_t* new_g = deserializeGraph("map.roadmap", "edges.txt", dimension);

	//create array of 5 query nodes
	state_node_t* query_nodes[nr_queries];
	for(int i = 0; i < nr_queries; i++){
		state_node_t* query_node = new state_node_t(dimension);

		for(int j=0; j<dimension; ++j){
				//create a random point between -2,2
				query_node->state[j] = uniform_random()*4-2;
		}

		query_nodes[i] = query_node;
	}
	
	for(int i = 0; i < nr_queries; i++){
		srand(i);
		print_node(g->find_closest(query_nodes[i],&dist1));
		srand(i);
		print_node(new_g->find_closest(query_nodes[i],&dist2));
		std::cout << "\n";
	}

	//serialize new graph in order to compare to original graph built
	serializeGraph(new_g, "edges2.txt");

	return 0;
}


gnn::graph_nearest_neighbors_t* buildGraph(std::string filename, int nr_dim){
	std::ifstream myfile;
	myfile.open(filename);
	int nr_nodes;

	//create empty graph
	gnn::graph_nearest_neighbors_t* g;
	g = new gnn::graph_nearest_neighbors_t(std::bind(&distance,std::placeholders::_1,std::placeholders::_2));

	if(myfile.is_open()){
		//first line of file is the number of nodes
		myfile >> nr_nodes;
		std::string line;
		//skip to the next line
		std::getline(myfile, line);

		//repeat for each node
		for(int i=0;i<nr_nodes;++i){
			int index;
			std::vector<double> state;

			//first number in the line is the node index
			myfile >> index;

			// get the rest of the line and parse it to get the state variables
			std::getline(myfile, line);
			int space = line.find(" " , 0);
			line = line.substr(space+1, line.length());

			std::stringstream ss(line);
			float temp;

			//store each state variable in a vector and skip commas
			while(ss >> temp){
				state.push_back(temp);
				if(ss.peek() == ',')
					ss.ignore();
			}

			auto new_node = new state_node_t(nr_dim);
			new_node->set_index(index);
			for(int j=0;j<nr_dim;++j)
			{
				//set node state variables using the temporary vector
				new_node->state[j] = state.at(j);

			}

			//add the node to the graph
			g->add_node(new_node);
		}

	}
	return g;
}


void serializeGraph(gnn::graph_nearest_neighbors_t* graph, std::string filename){
	std::ofstream myfile;
	myfile.open(filename);

	if(myfile.is_open()){
		//get number of nodes in graph and write it to the fie
		myfile << graph->get_nr_nodes() << "\n";

		gnn::proximity_node_t** nodes = graph->get_nodes();

		//for each node in the graph
		for(int i = 0; i < graph->get_nr_nodes(); i++){
			int nr_neigh;
			unsigned int* neighbors = (*nodes)->get_neighbors(&nr_neigh);

			//get node index and write to file
			myfile << (*nodes)->get_index() << " ";
			//write number of neighbors to the file
			myfile << nr_neigh << "\n";

			//for each of the neighbors
			for(int j = 0; j < nr_neigh; j++){
				//write the index of the neighbor as well as a comma
				//don't add comma after last neighbor
				myfile << (*neighbors) << ","; 

				//increment neighbor pointer
				neighbors++;
			}
			myfile << std::endl;

			//increment node pointer
			nodes++;
		}
	}
	myfile.close();
}



gnn::graph_nearest_neighbors_t* deserializeGraph(std::string nodeFile, std::string edgeFile, int nr_dim){
	std::ifstream nodesFile;
	nodesFile.open(nodeFile);
	int nr_nodes;

	//create new empty graph
	gnn::graph_nearest_neighbors_t* g;
	g = new gnn::graph_nearest_neighbors_t(std::bind(&distance,std::placeholders::_1,std::placeholders::_2));


	
	if(nodesFile.is_open()){
		//first line of file is the number of nodes
		nodesFile >> nr_nodes;

		//skip to the next line
		std::string line;
		std::getline(nodesFile, line);

		//Add all nodes to graph without connecting to neighbors
		for(int i=0;i<nr_nodes;++i){
			int index;
			std::vector<double> state;

			//first number in the line is the node index
			nodesFile >> index;

			// get the rest of the line and parse it to get the state variables
			std::getline(nodesFile, line);
			int space = line.find(" " , 0);
			line = line.substr(space+1, line.length());

			std::stringstream ss(line);
			float temp;

			//store each state variable in a vector and skip commas
			while(ss >> temp){
				//std::cout << temp << " ";
				state.push_back(temp);
				if(ss.peek() == ',')
					ss.ignore();
			}


			//create new empty node to add to add to graph
			auto new_node = new state_node_t(nr_dim);
			new_node->set_index(index);
			for(int j=0;j<nr_dim;++j)
			{
				//set node state variables using the temporary vector
				new_node->state[j] = state.at(j);
			}

			//use custom add function found in gnn.cpp
			g->add_node_deserialize(new_node);
		}
	}

	nodesFile.close();

	std::ifstream edgesFile;
	edgesFile.open(edgeFile);

	//Connect all nodes in the graph to their neighbors
	if(edgesFile.is_open()){

		//first line of file is the number of nodes
		edgesFile >> nr_nodes;


		//skip to the next line
		std::string line;
		std::getline(edgesFile, line);

		//for each node
		for(int i=0;i<nr_nodes;++i){
			int index;
			int nr_neighbors;

			std::getline(edgesFile, line);

			std::stringstream sss(line);
			//get node index
			sss >> index;
			sss.ignore();
			//get number of neighbors
			sss >> nr_neighbors;

			
			gnn::proximity_node_t* node;
			
			//get node pointer from graph based on the node index (custom)
			node = g->get_node_by_index(index);

			std::getline(edgesFile, line);
			std::stringstream ssr(line);

			int neigh;
			//add each neighbor to the node and skip commas
			while(ssr >> neigh){
				node->add_neighbor(neigh);
				if(ssr.peek() == ',')
					ssr.ignore();
			}
		}
	}

	edgesFile.close();
	return g;
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