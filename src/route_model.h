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
        
        void FindNeighbors();
        float distance(Node other) const { 
          return std::sqrt(std::pow((x - other.x), 2) + std::pow((y - other.y), 2)); 
        }
        Node(){}
        Node(int idx, RouteModel * search_model, Model::Node node) : Model::Node(node), parent_model(search_model), index(idx) {}
                      
      private:
      // Add private Node variables and methods here.
      int index;
      RouteModel* parent_model = nullptr;
      
      RouteModel::Node * FindNeighbor (std::vector<int> node_indices);      
    };
    
    // Add public RouteModel variables and methods here.
    std::vector<Node> path; // This variable will eventually store the path that is found by the A* search.

    RouteModel(const std::vector<std::byte> &xml);
    auto &GetNodeToRoadMap(){ return node_to_road; }
    auto &SNodes(){ return m_nodes;}
    RouteModel::Node &FindClosestNode(float x, float y); 
    
  private:
    // Add private RouteModel variables and methods here.
    std::vector<Node> m_nodes{};
    std::unordered_map<int, std::vector<const Model::Road*>> node_to_road;

    void CreateNodeToRoadHashmap();    
};

#endif