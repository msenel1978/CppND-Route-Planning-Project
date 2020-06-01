#include "route_planner.h"
#include "debug.h"
#include <algorithm>


#ifdef DEBUG_BUILD
// Count the number of iterations for DEBUG
static int debug_iteration = 0;
#endif

/**
 ** Compare h + g values of two cells.
 */
bool Compare(const RouteModel::Node *node1, RouteModel::Node *node2) {
    float f1 = node1->h_value + node1->g_value; // node1: h + g
    float f2 = node2->h_value + node2->g_value; // node2: h + g
    
    return f1 > f2;
}


RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y,\
				    float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // DONE 2: Use the m_Model.FindClosestNode method to find the closest
    // nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node
    // attributes.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);

}


// DONE 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another
// node.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return(node->distance(*end_node));	
}


// DONE 4: Complete the AddNeighbors method to expand the current node by
// adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate
// current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value,
// the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list
// and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // Populate current_node.neighbors vector with all the neighbors
    current_node->FindNeighbors();

    // For each node in current_node.neighbors
    for (RouteModel::Node *neighbor : current_node->neighbors) {
        // Set the parent, the h_value
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);

        // g_value is the total distance so far
        neighbor->g_value = current_node->g_value\
			    + neighbor->distance(*current_node);

        // Add the neighbor to open_list
        open_list.push_back(neighbor);

	  // Set the node's visited attribute to true
	  neighbor->visited = true;
    }
}


// DONE 5: Complete the NextNode method to sort the open list and return the
// next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.
RouteModel::Node *RoutePlanner::NextNode() {
    RouteModel::Node *min_node;

    // Sort open_list	  
    std::sort(open_list.begin(), open_list.end(), Compare);

#ifdef DEBUG_BUILD
    // Print the sorted list for DEBUG
    std::cout << "\nSorted open_list - Iteration " << debug_iteration ++ <<": ";
    for (RouteModel::Node *node : open_list)
	  std::cout << node->h_value + node->g_value << " ";
#endif

    // The node with the lowest g + h should be at the end
    // of sorted open_list
    min_node = open_list.back();
    DEBUG("\n#####\nmin_node: %f\n", min_node->h_value + min_node->g_value);

    // Remove the node from sorted open_list
    open_list.pop_back();

    return min_node;
}


// DONE 6: Complete the ConstructFinalPath method to return the final path
// found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and
//   iteratively follow the chain of parents of nodes until the starting node is
//   found.
// - For each node in the chain, add the distance from the node to its parent
//   to the distance variable.
// - The returned vector should be in the correct order: the start node should
//   be the first element  of the vector, the end node should be the last
//   element.

std::vector<RouteModel::Node> 
RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // DONE: Implement your solution here.
    // Iterate until current_node == start_node
    while (current_node != start_node) {
	  // Add to the beginning of path_found
	  path_found.insert(path_found.begin(), *current_node);
	  // Update distance
	  distance += (current_node->distance(*current_node->parent));
	  // Next node
	  current_node = current_node->parent;
    }
    // Add start_node to the beginning of path_found, as well
    path_found.insert(path_found.begin(), *current_node);

    // Multiply the distance by the scale of the map to get meters. 
    distance *= m_Model.MetricScale();

    return path_found;

}


// DONE 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current 
//   node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath
//   method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method
//   exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // DONE: Implement your solution here.
    current_node = start_node;
    // Make sure start_node is visited
    current_node->visited = true;

    // - Use the AddNeighbors method to add all of the neighbors of the current 
    //   node to the open_list.
    // - Use the NextNode() method to sort the open_list and
    //   return the next node.
    while (current_node != end_node) {
	  AddNeighbors(current_node);
	  current_node = NextNode();
    }

    // current_node = end_node
    // When the search has reached the end_node, use the ConstructFinalPath
    // method to store the final path in the m_Model.path attribute 
    m_Model.path = ConstructFinalPath(current_node);

}
