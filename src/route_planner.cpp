#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage: 
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
    
    // Find closest nodes to start and end coordinates
    this->start_node = &(m_Model.FindClosestNode(start_x, start_y));
    this->end_node = &(m_Model.FindClosestNode(end_x, end_y));
}

// Calculate Manhattan Distance to the end node in order to influence the next node that we choose
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return sqrt((node->x - end_node->x) * (node->x - end_node->x) + (node->y - end_node->y) * (node->y - end_node->y));
}

// Add neighbors using FindNeighbors function from RouteModel
// Then for each neighbor update properties
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();

    for (RouteModel::Node* neighbor : current_node->neighbors) {
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        neighbor->visited = true;
        this->open_list.push_back(neighbor);
    }
}

// helper function to sort nodes ascending by sum of current_distance + manhattan_distance to end_node
bool sortHelper(RouteModel::Node* i, RouteModel::Node* j) {
    return (i->g_value + i->h_value < j->g_value + j->h_value);
}

// sort the list of nodes and pop the node with the smallest distance
RouteModel::Node *RoutePlanner::NextNode() {

    std::sort(open_list.begin(), open_list.end(), sortHelper);

    RouteModel::Node* nextNode = open_list.front();
    open_list.erase(open_list.begin());

    // std::cout << "NextNode Function Returns!\n";
    return nextNode;
}

// Having stored the parent for each node, we can backtrack to the first_node and create a path
// Then we reverse the path in order to begin from the start_node

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while (current_node->parent != nullptr) {
        path_found.push_back(*current_node);
        distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
    }
    path_found.push_back(*current_node);

    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}

// Finally, we keep all the neighbors in a list beginning from start_node
// Each time extracting the best candidate
// When we reach the end_node we stop
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
    start_node->visited = true;
    open_list.push_back(start_node);

    while (open_list.size() > 0) {
        current_node = NextNode();

        if (current_node->distance(*end_node) == 0)
            break;

        // std::cout << "Adding neighbors of last node...\n";
        AddNeighbors(current_node);
    }

    m_Model.path = ConstructFinalPath(end_node);
}