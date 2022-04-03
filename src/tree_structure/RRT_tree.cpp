//
// Created by misha on 10/17/21.
//

#include "RRT_tree.h"


RRT_tree::RRT_tree(Vec3 &coords, World *ptr_to_world, double neighbor_radius) {

	root = std::make_shared<Node>(coords);
	world_p = ptr_to_world;
    this->neighbor_radius = neighbor_radius;
}

std::vector<Vec3> RRT_tree::find_way_from_goal_to_root(Node* last_goal_p) {
	std::vector<Vec3> way;
	way.push_back(last_goal_p->coords);

	Node *current_ptr_to_parent = last_goal_p->parent;
	while (current_ptr_to_parent != nullptr) {
		way.push_back(current_ptr_to_parent->coords);
		current_ptr_to_parent = current_ptr_to_parent->parent;
	}
	std::reverse(way.begin(), way.end());
	return way;
}

void RRT_tree::printout_the_path(const std::vector<Vec3> &path) {
    std::cout << "\nPath:\n";
    for (const Vec3 &point : path)
        point.printout();
    std::cout << std::endl;
}

