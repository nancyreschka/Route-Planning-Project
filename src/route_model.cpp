#include "route_model.h"
#include <iostream>

RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
  int count = 0;

  // create RouteModel Nodes
  for(Model::Node node : this->Nodes())
  {
     m_Nodes.push_back(Node(count, this, node));
     count++;       
  }
  CreateNodeToRoadHashmap();
}

void RouteModel::CreateNodeToRoadHashmap()
{
  for(const Model::Road &road : Roads())
  {
     if(road.type != Model::Road::Type::Footway)
     {
       for (int node_idx : Ways()[road.way].nodes)
       {
          if(node_to_road.find(node_idx) == node_to_road.end())
          {
            node_to_road[node_idx] = std::vector<const Model::Road*>();
          }
          node_to_road[node_idx].push_back(&road);
       }
     }
  }
} 

RouteModel::Node *RouteModel::Node::FindNeighbor (std::vector<int> node_indices)
{
  Node *closest_node = nullptr;
  Node node;

  for (auto index : node_indices)
  {
    node = parent_model->SNodes()[index];
    if((!node.visited) && (this->distance(node) != 0))
    {
      if((closest_node == nullptr) || (this->distance(node) < this->distance(*closest_node)))
      {
        closest_node = &parent_model->SNodes()[index];
      }      
    }
  }

  return closest_node;  
}

void RouteModel::Node::FindNeighbors()
{
  for(auto &road : parent_model->node_to_road[this->index])
  {
    RouteModel::Node *newNeighbor = this->FindNeighbor(parent_model->Ways()[road->way].nodes);
    if(newNeighbor)
    {
      this->neighbors.emplace_back(newNeighbor);
    }
  }
}

RouteModel::Node &RouteModel::FindClosestNode(float x, float y)
{
  Node tempNode;
  tempNode.x = x;
  tempNode.y = y;
  
  float min_dist = std::numeric_limits<float>::max();
  float dist;
  int closest_idx;

  for(const Model::Road &road : Roads())
  {
    if(road.type != Model::Road::Type::Footway)
    {
      for(int nodeIndex : Ways()[road.way].nodes)
      {
        dist = tempNode.distance(SNodes()[nodeIndex]);
        if(dist < min_dist)
        {
          min_dist = dist;
          closest_idx = nodeIndex;
        }
      }
    }
  }
  return SNodes()[closest_idx];
}