//
// Created by misha on 10/17/21.
//

#include "RRT_tree.h"

RRT_tree::RRT_tree(Vec3 &coords, World *ptr_to_world) {
	root = std::make_shared<Node>(coords);
	world_p = ptr_to_world;
}

std::vector<Vec3> RRT_tree::find_way_from_goal_to_root(Node* last_goal_p) {
	std::vector<Vec3> way;
	way.push_back(last_goal_p->coords);
	std::cout << "adding: " << last_goal_p->coords.x << "\n";
	
	Node *current_ptr_to_parent = last_goal_p->parent;
	while (current_ptr_to_parent != nullptr) {
		way.push_back(current_ptr_to_parent->coords);
		std::cout << "adding: " << current_ptr_to_parent->coords.x << "\n";
		current_ptr_to_parent = current_ptr_to_parent->parent;
	}
	std::reverse(way.begin(), way.end());
	return way;
}


//

std::vector<Vec3> RRT_tree::find_path_to_goal(World *world_ptr, Vec3& start_point, Vec3& goal_point) {
	RRT_tree tree(start_point, world_ptr);
	
	Vec3 center = (start_point + goal_point) / 2.0;
	
	double dist_to_goal = Vec3::distance_between_two_vec3(start_point, goal_point);
	double i = -1.0;
	
	while (true) {
		i += 0.1;
		if (fabs(i) <= 0.01) continue;
		
//		Vec3 rnd_point = Vec3(i, 0.0, 0.0);
		Vec3 rnd_point = Vec3::random_vec3(-5.0, center.x + dist_to_goal,
		                                   0, 0,
		                                   0, 0);
		
		auto closest = Node::find_the_closest_node(rnd_point, tree.root.get());
		closest->add_child(rnd_point);
		closest->print_out_all_children();
		if (Vec3::distance_between_two_vec3(rnd_point, goal_point) < 2.0) {
			return tree.find_way_from_goal_to_root(closest->children.back().get());
		}
	}
}

