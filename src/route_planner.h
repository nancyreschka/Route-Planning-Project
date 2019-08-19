#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H

#include <iostream>
#include <vector>
#include <string>
#include "route_model.h"

// perform A* Search
class RoutePlanner {
  public:
    // Constructor gets the RouteModel the A* search shall be performed on and
    // with the floats the start_node and end_node are constructed
    // floats are scaled to percentages by multiplying each with 0.01
    RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y);
    // Add public variables or methods declarations here.
    // total distance for the route that A* search finds from start_node to end_node
    float GetDistance() const { return distance; };
    // perform A* search
    void AStarSearch();

  private:
    // Add private variables or methods declarations here.
    // reference to the model that A* search will be performed on
    RouteModel &m_Model;
    // node in the model which is closest to our starting point
    RouteModel::Node* start_node;
    // node in the model which is closest to our ending point
    RouteModel::Node* end_node;
    // total distance for the route that A* search finds from start_node to end_node
    float distance;
    // list of open nodes
    std::vector<RouteModel::Node*> open_list;
    // reconstruct the final sequence of nodes found from the start_node to the end_node, 
    // so that the A* search can store the sequence
    // starting with the last node that was found, and then iteratively traversing to the parent of that node 
    // until start_node  is found
    std::vector<RouteModel::Node> ConstructFinalPath(RouteModel::Node *current_node);
    // calculates h-value for the given node
    // h-value will be computed as the euclidean distance from the node to the end node
    float CalculateHValue(const RouteModel::Node *node);
    // sort the list of open nodes in the A* search, return the node with the lowest f-value, 
    // and remove the node from the list of open nodes
    RouteModel::Node *NextNode();
    // take each neighbor of the current node in the A* search, set the neighbor's g-value, 
    // h-value, and parent, and add the neighbor to the open list.
    void AddNeighbors(RouteModel::Node *current_node);    
};

#endif