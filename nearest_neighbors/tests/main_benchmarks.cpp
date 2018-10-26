
#include "gnn.hpp"

#include <sys/time.h>

#include <iostream>
#include <vector>
#include <sstream>
#include <fstream>
#include <chrono>

static const int NUM_NODES = 10000;

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

void print_node(gnn::proximity_node_t* node)
{
    auto n = (state_node_t*)node;
    for(int i=0;i<n->d-1;++i)
    {
        std::cout<<n->state[i]<<", ";
    }
    std::cout<<n->state[n->d-1]<<std::endl;
}

std::vector<double> get_point_from_line(std::string line)
{
    std::vector<double> point;
    std::string temp;
    std::stringstream input_stream(line);
    
    while(getline(input_stream,temp, ','))
    {
        point.push_back(stod(temp));
    }
    
    return point;
}

int main(int argc, char* argv[])
{
    std::fstream input_roadmap_file("../tests/data/n10000.roadmap");
    
    if (!input_roadmap_file)
    {
        std::cout << "Unable to open file"<<std::endl;
        return -1;
    }
    
    std::string file_line;
    std::string file_line_number;
    const int dimension = 9;
    gnn::graph_nearest_neighbors_t* g;
    std::vector<double> temp_point;
    
    g = new gnn::graph_nearest_neighbors_t(std::bind(&distance,std::placeholders::_1,std::placeholders::_2));
    
    std::vector<state_node_t*> all_nodes;
    state_node_t* query_node = new state_node_t(dimension);
    
    input_roadmap_file >> file_line_number;
    
    for(int i=0;i<NUM_NODES-1;++i)
    {
        input_roadmap_file >> file_line_number;
        file_line = "";
        input_roadmap_file >> file_line;
    
        temp_point = get_point_from_line(file_line);
        auto new_node = new state_node_t(dimension);
        for(int j=0;j<dimension;++j)
        {
            //set the node as read from file
            new_node->state[j] =  temp_point[j];
        }
        g->add_node(new_node);
        all_nodes.push_back(new_node);
    }
    
    input_roadmap_file >> file_line_number;
    input_roadmap_file >> file_line;
    
    temp_point = get_point_from_line(file_line);
    
    for(int j=0;j<dimension;++j)
    {
        //set the last point in the file as query point
        query_node->state[j] = temp_point[j];
    }
    
    input_roadmap_file.close();
    
    double dist;
    int k_required = 10;
    double delta_close_distances[10];
    
    state_node_t** k_close_nodes;
    
    k_close_nodes = new state_node_t*[10];
//    for(int it=0; it<k_required; ++it)
//    {
//        k_close_nodes[it] = new state_node_t(dimension);
//    }
//
    std::cout<<"Query Node: ";
    print_node(query_node);
    std::cout<<"Closest Node: ";
    
    std::chrono::high_resolution_clock::time_point start_time, end_time;
    
    //benchmarking for find_closest() method
    start_time = std::chrono::high_resolution_clock::now();
    print_node(g->find_closest(query_node,&dist));
    end_time = std::chrono::high_resolution_clock::now();
    
    auto execution_time = std::chrono::duration_cast<std::chrono::microseconds>( end_time - start_time ).count();
    std::cout<<"Time taken for find_closest() method in microseconds: "<<execution_time<<std::endl;
    //
    
    
    //benchmarking for find_k_close() method
    start_time = std::chrono::high_resolution_clock::now();
    int k_found = g->find_k_close(query_node,(gnn::proximity_node_t **)k_close_nodes, &dist, 9);
    end_time = std::chrono::high_resolution_clock::now();
    std::cout<<"Nodes returned : "<<k_found<<std::endl;
    
    execution_time = std::chrono::duration_cast<std::chrono::microseconds>( end_time - start_time ).count();
    std::cout<<"Time taken for find_k_close() method in microseconds: "<<execution_time<<std::endl;
    //
    
    //benchmarking for find_k_close() method
    start_time = std::chrono::high_resolution_clock::now();
    k_found = g->find_delta_close(query_node,(gnn::proximity_node_t **)k_close_nodes, delta_close_distances, 1.0);
    end_time = std::chrono::high_resolution_clock::now();
    std::cout<<"Nodes returned : "<<k_found<<std::endl;
    
    execution_time = std::chrono::duration_cast<std::chrono::microseconds>( end_time - start_time ).count();
    std::cout<<"Time taken for find_delta_close() method in microseconds: "<<execution_time<<std::endl;
    //
    
    //cleanup
    for(int i=0;i<NUM_NODES-1;++i)
    {
        g->remove_node(all_nodes[i]);
    }
    
    delete query_node;
    delete g;
    
//    for(int i=0;i<k_required;++i)
//    {
//        delete k_close_nodes[i];
//    }
    delete[] k_close_nodes;
    //
    
    
    return 0;
}
