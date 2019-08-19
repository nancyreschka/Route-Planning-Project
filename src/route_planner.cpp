#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Scale the floats to percentages
    start_x *= 0.01;
    start_y *= 0.01;
    // find the closest node on the map to the given values
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    
    // Scale the floats to percentages
    end_x /= 100;
    end_y /= 100;
    // find the closest node on the map to the given values
    end_node = &m_Model.FindClosestNode(end_x, end_y);    
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node)
{
    std::vector<RouteModel::Node> path_found;
    RouteModel::Node parent;
    distance = 0.0f;

    // iterate through the node parents until a node with parent equal to nullptr is reached
    // this will be the start node, which has no parent
    while(current_node->parent != nullptr)
    {
        // each node is stored in found_path
        path_found.push_back(*current_node);
        parent = *(current_node->parent);
        // add the distance between a node and its parent to the class distance variable
        distance += current_node->distance(parent);
        current_node = current_node->parent;
    }
    path_found.push_back(*current_node);
    // scale the distance to metric scale
    // This is done since node coordinates are scaled down when they are stored in the model.
    // They must be rescaled to get an accurate distance.
    distance *= m_Model.MetricScale();
    return path_found;
}

void RoutePlanner::AStarSearch()
{
    start_node->visited = true;
    open_list.push_back(start_node);
    RouteModel::Node *current_node = nullptr;

    while(open_list.size() > 0)
    {
        current_node = NextNode();
        // if the distance from current_node to the end_node is 0     
        if(current_node->distance(*end_node) == 0)
        {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        // add the current node to neighbors
        AddNeighbors(current_node);
    }
}

float RoutePlanner::CalculateHValue(const RouteModel::Node *node)
{
    // return the distance for the passed argument to the end_node
    return (*node).distance(*end_node);
}

RouteModel::Node *RoutePlanner::NextNode()
{
    // sort the open_list according to the f-value, which is the sum of a node's h-value and g-value
    std::sort(open_list.begin(), open_list.end(), [](const auto &_1st, const auto &_2nd) {
        return _1st->g_value + _1st->h_value < _2nd->g_value + _2nd->h_value;
    });
    // create a copy of the pointer to the node with the lowest f-value
    RouteModel::Node *found_node = open_list.front();
    // Erase that node pointer from open_list
    open_list.erase(open_list.begin());
    // return the pointer copy
    return found_node;
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node)
{
    // populate the current_node's neighbors vector.
    current_node->FindNeighbors();

    // for each neighbor set the parent, g-value and h-value
    for(RouteModel::Node* neighbor : current_node->neighbors)
    {
        neighbor->parent = current_node;
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        neighbor->h_value = CalculateHValue(neighbor);
        // store the neighbor in open_list
        open_list.push_back(neighbor);
        // mark the neighbor as visited
        neighbor->visited = true;        
    }
}