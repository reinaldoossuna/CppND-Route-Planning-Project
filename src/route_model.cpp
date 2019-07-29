#include "route_model.h"
#include <iostream>

RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {

  int counter =0;
  for(Model::Node node : this->Nodes()){
    m_Nodes.push_back(Node(counter,this,node));
    counter++;
  }
  CreateNodeToRoadHashmap();
}

void RouteModel::CreateNodeToRoadHashmap(){

  // Loop through the roads
  for (const Model::Road &road : this->Roads() ) {
    // check if the road is not a footway
    if (road.type != Model::Road::Type::Footway ) {
      // loop through all nodes in way
      for (int node_idx : this->Ways()[road.way].nodes) {
        // check if the noad is in the hashmap, if not create a empyt vector
        if (node_to_road.find(node_idx) == node_to_road.end())
          node_to_road[node_idx] = std::vector<const Model::Road *> ();

        // push back pointer to the road in the hashmap
        node_to_road[node_idx].push_back(&road);
      }
    }
  }
}

RouteModel::Node* RouteModel::Node::FindNeighbor(std::vector<int> node_indices){
  Node *closest_node = nullptr;
  Node inTest;

  for (int idx: node_indices) {
    inTest = parent_model->SNodes().at(idx);
    if (!inTest.visited && this->distance(inTest) != 0) {
      if (closest_node == nullptr || this->distance(inTest) < this->distance(*closest_node)){  
        closest_node = &(parent_model->SNodes().at(idx) );
      }
    }
  }
  return closest_node;
}

void RouteModel::Node::FindNeighbors() {
  for (auto & road: parent_model->node_to_road[this->index]) {
    RouteModel::Node* new_neighbor = this->FindNeighbor(parent_model->Ways()[road->way].nodes);
    if (new_neighbor)
      (this->neighbors).emplace_back(new_neighbor);
  }
}

RouteModel::Node& RouteModel::FindClosestNode(float x,float y) {
  Node input;
  input.x = x;
  input.y = y;

  Node inTest;

  float min_dist = std::numeric_limits<float>::max();
  int closest_idx;

  for (const Model::Road road: this->Roads()) {
    if (road.type != Model::Road::Type::Footway) {
      for (int node_idx: this->Ways()[road.way].nodes) {
        inTest = this->SNodes().at(node_idx);
        if (inTest.distance(input) < min_dist) {
          min_dist = inTest.distance(input);
          closest_idx = node_idx;
        }
      }
    }
  }
  return this->SNodes().at(closest_idx);
}
