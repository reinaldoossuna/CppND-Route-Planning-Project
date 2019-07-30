#include "route_planner.h"
#include <algorithm>
#include <vector>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {

  start_x *= 0.01;
  start_y *= 0.01;
  end_x *= 0.01;
  end_y *= 0.01;


  start_node = &(m_Model.FindClosestNode(start_x, start_y));

  end_node = &(m_Model.FindClosestNode(end_x, end_y));

}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
  std::vector<RouteModel::Node> path_found;
  this->distance = 0.0f;
  RouteModel::Node *parent;

  while(current_node->parent != nullptr ) {
    path_found.push_back(*current_node);
    parent = current_node->parent;
    this->distance += current_node->distance(*parent);

    current_node = parent;
  }
  path_found.push_back(*current_node);

  this->distance *= m_Model.MetricScale();
  return path_found;
}

void RoutePlanner::AStarSearch() {
  start_node->visited = true;
  open_list.push_back(start_node);
  RouteModel::Node *current_node = nullptr;

  while(!open_list.empty()) {
    current_node = NextNode();

    if (current_node->distance(*end_node) == 0) {
      m_Model.path = ConstructFinalPath(current_node);
      return;
    }
    //ELSE
    AddNeighbors(current_node);
  }
}

float RoutePlanner::CalculateHValue(const RouteModel::Node *node) {
  return node->distance(*end_node);
}

bool RoutePlanner::Compare(RouteModel::Node *first, RouteModel::Node *second){
  float f_fisrt = first->g_value + first->h_value;
  float f_second = second->g_value + second->h_value;
  return f_fisrt > f_second;
}

RouteModel::Node* RoutePlanner::NextNode() {
  std::sort(open_list.begin(),open_list.end(),Compare);
  RouteModel::Node *nextNode = open_list.back();

  open_list.pop_back();
  return nextNode;
}
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  current_node->FindNeighbors();
  for (auto neighbor: current_node->neighbors ) {
    neighbor->parent = current_node;
    neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
    neighbor->h_value = CalculateHValue(neighbor);

    open_list.insert(open_list.begin(), neighbor);
    neighbor->visited = true;
  }
}
