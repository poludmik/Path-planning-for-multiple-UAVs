//
// Created by misha on 10/12/21.
//

#include "Node.h"

Node::Node(Vec3 &point, Node *parent) {
	this->coords = point;
	this->parent = parent;
}

Node::Node(Vec3 &point) {
	this->coords = point;
	this->parent = nullptr;
}

void Node::add_child(Vec3 &point) {
	children.emplace_back(std::make_shared<Node>(point, this));
}

void Node::print_out_all_children() {
	std::cout << "\nPrinting out all children of " << coords.x << ", " << coords.y << ", " << coords.z << std::endl;;
	for (auto & i : children){
		std::cout << i->coords.x << ", " << i->coords.y << ", " << i->coords.z << std::endl;
	}
}

std::shared_ptr<Node> Node::find_the_closest_node(Vec3 &point, Node *root_node) {
	
	double min_distance = 9999999999999.0;
	std::shared_ptr<Node> nearest_node_p = std::make_shared<Node>(*root_node);
	
	std::queue<std::shared_ptr<Node>> my_queue;
	
	min_distance = std::min(min_distance, Vec3::distance_between_two_vec3(root_node->coords, point));
	
	for (auto & i : root_node->children){
		my_queue.push(i);
	}
	
	while (!my_queue.empty()) {
		std::shared_ptr<Node> current = my_queue.front();
		double tmp_dist = Vec3::distance_between_two_vec3(current->coords, point);
		if (tmp_dist < min_distance){
			min_distance = tmp_dist;
			nearest_node_p = current;
		}
		my_queue.pop();
		for (auto & i : current->children){
			my_queue.push(i);
		}
	}
	
	return nearest_node_p;
}

