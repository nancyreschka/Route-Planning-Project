#include "route_model.h"
#include <iostream>

/** Constructor of RouteModel-Class
 * calls the Model constructor with the open street map data
 * creates new RouteModel::Node objects in m_Nodes
 */
RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
  std::vector<Model::Node> vectorGivenByThisNodes = this->Nodes();
  
  // create RouteModel Nodes
  for(int count = 0; count < vectorGivenByThisNodes.size(); count++)
  {
     m_Nodes.push_back(Node(count, this, vectorGivenByThisNodes[count]));
  }
  // creat the search path to follow the roads that the nodes are on
  CreateNodeToRoadHashmap();
}

void RouteModel::CreateNodeToRoadHashmap()
{
  // iterate through all Roads
  for(const Model::Road &road : Roads())
  {
    //check if it's not a footway
     if(road.type != Model::Road::Type::Footway)
     {
       // loop over each node_idx in the way that the road belongs to
       for (int node_idx : Ways()[road.way].nodes)
       {
          // if the node index is not in the node_to_road hashmap yet,
          // set the value for the node_idx key to be an empty vector of const Model::Road* objects         
          if(node_to_road.find(node_idx) == node_to_road.end())
          {
            node_to_road[node_idx] = std::vector<const Model::Road*>();
          }
          // push a pointer to the current road to the back of the vector 
          // given by the node_idx key in node_to_road
          node_to_road[node_idx].push_back(&road);
       }
     }
  }
} 

RouteModel::Node *RouteModel::Node::FindNeighbor (std::vector<int> node_indices)
{
  Node *closest_node = nullptr;
  Node node;

  // loop through the node_indices vector to find the closest unvisited node
  for (auto index : node_indices)
  {
    node = parent_model->SNodes()[index];
    // check that the node has not been visted and that the distance to this is nonzero
    if((!node.visited) && (this->distance(node) != 0))
    {
      // update closest_node if it's the first node or if the distance to 
      // this node is smaller than the distance to the closest_node
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
  // for each road that the current node belongs
  for(auto &road : parent_model->node_to_road[this->index])
  {
    // get the closest node
    RouteModel::Node *newNeighbor = this->FindNeighbor(parent_model->Ways()[road->way].nodes);
    // if a new neighbor is found, add it to neighbors
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
  
  // minimum distance found in search
  float min_dist = std::numeric_limits<float>::max();
  float dist;
  // index of the closest node
  int closest_idx;

  // for each road
  for(const Model::Road &road : Roads())
  {
    // check that it's not a footway
    if(road.type != Model::Road::Type::Footway)
    {
      /// for each node on the way
      for(int nodeIndex : Ways()[road.way].nodes)
      {
        // calculate distance to this node
        dist = tempNode.distance(SNodes()[nodeIndex]);
        // update distance if a closer node is found
        if(dist < min_dist)
        {
          min_dist = dist;
          closest_idx = nodeIndex;
        }
      }
    }
  }
  // return the closest node in SNodes with the found index
  return SNodes()[closest_idx];
}