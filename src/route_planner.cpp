#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    m_Model = model;
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    std::cout << "g value" << current_node->g_value;
    for (auto neighbor: current_node->neighbors){
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        open_list.emplace_back(neighbor);
        neighbor->visited = true;
    }
    
}

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), [ ]( const RouteModel::Node* lhs, const RouteModel::Node* rhs )
    {
        return (lhs->h_value + lhs->g_value) < (rhs->h_value + rhs->g_value);
    });
    auto next_node = open_list.front();
    open_list.erase(open_list.begin());
    return next_node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    path_found.emplace(path_found.begin(), *current_node);

    while (!(path_found.front() == *start_node))
    {
        distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
        path_found.emplace(path_found.begin(), *current_node);
    }

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = start_node;
    current_node->visited = true;
    AddNeighbors(current_node);

    while (!open_list.empty())
    {
        current_node = NextNode();        
        AddNeighbors(current_node);

        if (*current_node == *end_node)
        {
           m_Model.path = ConstructFinalPath(current_node);
           return;
        }
    }
}