#ifndef ROUTE_MODEL_H
#define ROUTE_MODEL_H

#include <limits>
#include <cmath>
#include <unordered_map>
#include "model.h"
#include <iostream>

class RouteModel : public Model {

  public:
    class Node : public Model::Node {
      public:
        // Add public Node variables and methods here.
        // node's parent
        RouteModel::Node* parent = nullptr;
        // node's h-value
        float h_value = std::numeric_limits<float>::max();
        // node's g-value
        float g_value = 0.0;
        // indicate if the node has been visited.
        bool visited = false;
        // node's neighbors
        std::vector<RouteModel::Node*> neighbors{};
        // find the closest neighbor from each road that the current node belongs to
        void FindNeighbors();
        // calculate the euclidean distance from the current node to the node passed in
        float distance(Node other) const { 
          return std::sqrt(std::pow((x - other.x), 2) + std::pow((y - other.y), 2)); 
        }
        Node(){}
        Node(int idx, RouteModel * search_model, Model::Node node) : Model::Node(node), parent_model(search_model), index(idx) {}
                      
      private:
      // Add private Node variables and methods here.
      int index;
      RouteModel* parent_model = nullptr;
      // return a pointer to the closest unvisited node from a vector of node indices,
      // where the distance is measured to the current node
      RouteModel::Node * FindNeighbor (std::vector<int> node_indices);      
    };
    
    // Add public RouteModel variables and methods here.
    std::vector<Node> path; // This variable will eventually store the path that is found by the A* search.

    RouteModel(const std::vector<std::byte> &xml);
    // get Node index values to the vector of Road pointers that those nodes belong to
    auto &GetNodeToRoadMap(){ return node_to_road; }
    // returns all nodes from the Open Street Map data
    auto &SNodes(){ return m_Nodes;}
    // find the nodes in the RouteModel that are closest to
    // the starting and ending coordinates given by the user
    // that is not a footway
    RouteModel::Node &FindClosestNode(float x, float y); 
    
  private:
    // Add private RouteModel variables and methods here.
    // stores all of the nodes from the Open Street Map data
    std::vector<Node> m_Nodes{};
    // hash table of Node index values to a vector of Road pointers that those nodes belong to
    std::unordered_map<int, std::vector<const Model::Road*>> node_to_road;
    /* for each road in Roads(), if it's not a Footway,
    Loop over each node_idx in the way that the road belongs to: Ways()[road.way].nodes
    add Node if not yet in note_to_road
    otherwise add pointer to current road 
     */
    void CreateNodeToRoadHashmap();    
};

#endif