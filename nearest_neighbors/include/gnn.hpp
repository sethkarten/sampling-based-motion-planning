/**
 * @file gnn.hpp 
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2017, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#include <functional>
#include <cmath>

#ifndef GNN_NEAREST_NEIGHBOR_HPP
#define GNN_NEAREST_NEIGHBOR_HPP

namespace gnn 
{ 
    #define MAX_KK            2000
    #define INIT_NODE_SIZE    1000
    #define INIT_CAP_NEIGHBORS 200

    /**
    * @brief <b> Proximity node for the graph-based distance metric.</b>
    * 
    * Proximity node for the graph-based distance metric. 
    * 
    * @author Kostas Bekris
    */
    class proximity_node_t
    {
    public:
        /**
        * @brief Constructor
        * @param st The node to store.
        */
        proximity_node_t();
        virtual ~proximity_node_t();

        /**
        * Gets the position of the node in the data structure. Used for fast deletion.
        * @brief Gets the position of the node in the data structure.
        * @return The index value.
        */
        int get_index();

        /**
        * Sets the position of the node in the data structure. Used for fast deletion.
        * @brief Sets the position of the node in the data structure.
        * @param indx The index value.
        */
        void set_index( int indx );

        /**
        * Returns the stored neighbors.
        * @brief Returns the stored neighbors.
        * @param nr_neigh Storage for the number of neighbors returned.
        * @return The neighbor indices.
        */
        unsigned int* get_neighbors( int* nr_neigh );

        /**
        * Adds a node index into this node's neighbor list.
        * @brief Adds a node index into this node's neighbor list.
        * @param node The index to add.
        */
        void add_neighbor( unsigned int node );

        /**
        * Deletes a node index from this node's neighbor list.
        * @brief Deletes a node index from this node's neighbor list.
        * @param node The index to delete.
        */
        void delete_neighbor( unsigned int node );

        /**
        * Replaces a node index from this node's neighbor list.
        * @brief Replaces a node index from this node's neighbor list.
        * @param prev The index to look for.
        * @param new_index The index to replace with.
        */
        void replace_neighbor( unsigned prev, int new_index );      

        unsigned added_index;

    protected:

        /**
        * @brief Index in the data structure. Serves as an identifier to other nodes.
        */
        int index;


        /**
        * @brief The max number of neighbors.
        */
        int cap_neighbors;

        /**
        * @brief The current number of neighbors.
        */
        int nr_neighbors;

        /**
        * @brief The neighbor list for this node.
        */
        unsigned int* neighbors;
    };

    /**
     * A proximity structure based on graph literature. Each node maintains a list of neighbors.
     * When performing queries, the graph is traversed to determine other locally close nodes.
     * @brief <b> A proximity structure based on graph literature. </b>
     * @author Kostas Bekris
     */
    class graph_nearest_neighbors_t
    {
    public:
        /**
        * @brief Constructor
        * @param state The first node to add to the structure.
        */
        graph_nearest_neighbors_t(std::function<double (proximity_node_t*,proximity_node_t*)>);
        ~graph_nearest_neighbors_t();

        /**
        * Adds a node to the proximity structure
        * @brief Adds a node to the proximity structure
        * @param node The node to insert.
        */
        void add_node( proximity_node_t* node );

        /**
        * Adds a node to the proximity structure
        * @brief Adds a node to the proximity structure
        * @param node The node to insert.
        */
        void add_nodes( proximity_node_t** nodes, int nr_nodes );

        /**
        * @brief Removes a node from the structure.
        * @param node
        */
        void remove_node( proximity_node_t* node );

        /**
        * Prints the average degree of all vertices in the data structure.
        * @brief Prints the average degree of all vertices in the data structure.
        */
        void average_valence();

        /**
        * Returns the closest node in the data structure.
        * @brief Returns the closest node in the data structure.
        * @param state The query point.
        * @param distance The resulting distance between the closest point and the query point.
        * @return The closest point.
        */
        proximity_node_t* find_closest( proximity_node_t* state, double* distance );          

        /**
        * Find the k closest nodes to the query point. 
        * @brief Find the k closest nodes to the query point.
        * @param state The query state.
        * @param close_nodes The returned close nodes.
        * @param distances The corresponding distances to the query point.
        * @param k The number to return.
        * @return The number of nodes actually returned.
        */
        int find_k_close( proximity_node_t* state, proximity_node_t** close_nodes, double* distances, int k );

        /**
        * Find all nodes within a radius and the closest node. 
        * @brief Find all nodes within a radius and the closest node.
        * @param state The query state.
        * @param close_nodes The returned close nodes.
        * @param distances The corresponding distances to the query point.
        * @param delta The radius to search within.
        * @return The number of nodes returned.
        */
        int find_delta_close_and_closest( proximity_node_t* state, proximity_node_t** close_nodes, double* distances, double delta );

        /**
        * Find all nodes within a radius. 
        * @brief Find all nodes within a radius.
        * @param state The query state.
        * @param close_nodes The returned close nodes.
        * @param distances The corresponding distances to the query point.
        * @param delta The radius to search within.
        * @return The number of nodes returned.
        */
        int find_delta_close( proximity_node_t* state, proximity_node_t** close_nodes, double* distances, double delta );

        /**
        * Add node to graph without adding neighbors.
        * @param graph_node The node to be added to the graph.
        */ 
        void add_node_deserialize( proximity_node_t* graph_node );
        
        /**
        * Return pointer to node given a specific index.
        * @param index The index of the node to be returned.
        */ 
        proximity_node_t* get_node_by_index(int index);
        
        /**
        * Get function for number of nodes
        * @return Number of nodes in graph
        */ 
        int get_nr_nodes(){
            return nr_nodes;
        }

        /**
        * Get function for nodes
        * @return Graph's nodes
        */ 
        proximity_node_t** get_nodes(){
            return nodes;
        }

        /**
        * Determine the number of strongly connected components on the graph using DFS utility.
        * @return The number of strongly connected components.
        */
        int connectedComponents();

        /**
        * Count the total number of edges on the graph
        * @return The number of edges.
        */
        int count_edges();

        /**
        * Determine the minimun number of edges between two nodes
        * @param The start node
        * @param The goal node
        * @return The number of edges.
        */
        int minEdgeBFS(proximity_node_t* start_node, proximity_node_t* goal_node);



    protected:
        std::function<double (proximity_node_t*,proximity_node_t*)> distance_function;

        /**
        * Determine the number of nodes to sample for initial populations in queries.
        * @brief Determine the number of nodes to sample for initial populations in queries.
        * @return The number of random nodes to initially select.
        */
        inline int sampling_function()
        {
            if( nr_nodes < 500 )
                return nr_nodes/5 + 1;
            else
                return 100 + nr_nodes/500;
        }

        /**
        * Given the number of nodes, get the number of neighbors required for connectivity (in the limit).
        * @brief Given the number of nodes, get the number of neighbors required for connectivity (in the limit).
        * @return 
        */
        inline int percolation_threshold()
        {
            if( nr_nodes > 12)
                return( 3.5 * log( nr_nodes ));
            else 
                return nr_nodes;
        }

        /**
        * Sorts a list of proximity_node_t's. Performed using a quick sort operation. 
        * @param close_nodes The list to sort.
        * @param distances The distances that determine the ordering.
        * @param low The lower index.
        * @param high The upper index.
        */
        void sort_proximity_nodes( proximity_node_t** close_nodes, double* distances, int low, int high );

        /**
        * Performs sorting over a list of nodes. Assumes all nodes before index are sorted.
        * @param close_nodes The list to sort.
        * @param distances The distances that determine the ordering.
        * @param index The index to start from.
        */
        int resort_proximity_nodes( proximity_node_t** close_nodes, double* distances, int index );

        /**
        * Helper function for determining existance in a list.
        * @brief Helper function for determining existance in a list.
        * @param query_node The node to search for.
        * @param node_list The list to search.
        * @param list_size The size of the list.
        * @return If query_node exists in node_list.
        */
        bool does_node_exist( proximity_node_t* query_node );


        /**
        * The basic search process for finding the closest node to the query state.
        * @brief Find the closest node to the query state.
        * @param state The query state.
        * @param distance The corresponding distance to the query point.
        * @param node_index The index of the returned node.
        * @return The closest node.
        */
        proximity_node_t* basic_closest_search( proximity_node_t* state, double* distances, int* node_index );

        void clear_added();
        
        /**
        * Helper function to run Depth First Seacrh on the graph.
        * @brief Ru Depth First Seacrh on the graph.
        * @param graph_node The node to start DFS on.
        * @param visited The array to keep track of the visited nodes.
        */
        void DFSUtil(proximity_node_t* graph_node, bool visited[]);

        /**
        * @brief The nodes being stored.
        */
        proximity_node_t** nodes;

        /**
        * @brief The current number of nodes being stored.
        */
        int nr_nodes;

        /**
        * @brief The maximum number of nodes that can be stored. 
        */
        int cap_nodes;

        /**
        * @brief Temporary storage for query functions.
        */
        proximity_node_t** second_nodes;

        /**
        * @brief Temporary storage for query functions.
        */
        double* second_distances;

        // std::vector<proximity_node_t*> added_nodes;

        unsigned added_node_id;
    };
}

#endif