#include <iostream>
#include <vector>

#include "planning/rrt.h"

int main() {
    Node* start = new Node(1.0, 1.0);
    Node* goal = new Node(7.0, 4.0);

    std::vector<std::vector<float>> obstacles {
        {1.0, 2.0, 2.0, 5.0},   // First obstacle
        {2.0, 2.0, 2.5, 3.0},   // Second obstacle
        {6.0, 1.0, 8.0, 2.0},   // Third obstacle
        {4.0, 1.5, 5, 4.0},     // Fourth obstacle
        {2.0, 0.0, 3.5, 0.5} }; // Fifth obstacle

    RRT rrt(start, goal, obstacles, 1000, 0.1, 0.03, 0.1, 100.0);
    std::vector<Node*> final_path = rrt.planning();

    // Example: print planned path node by node (+ cost)
    /*
    if (!final_path.empty()) {
        for (Node* node : final_path) {
            std::cout << node->x << ", " << node->y << ", " << node->cost << std::endl;
        }
        std::cout << goal->x << ", " << goal->y << ", " << goal->cost << std::endl;
    }
    */
}
