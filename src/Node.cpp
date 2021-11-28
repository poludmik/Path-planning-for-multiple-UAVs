//
// Created by misha on 10/12/21.
//

#include "Node.h"

Node::Node(const Vec3 &point, Node *parent) {
	this->coords = point;
	this->parent = parent;
    cost = parent->cost + Vec3::distance_between_two_vec3(point, parent->coords);
}

Node::Node(const Vec3 &point) {
	this->coords = point;
	this->parent = nullptr;
    cost = 0;
}

void Node::add_child(const Vec3 &point) {
	children.emplace_back(std::make_shared<Node>(point, this));
}

void Node::print_out_all_children() const {
	std::cout << "\nPrinting out all children of " << coords.x << ", " << coords.y << ", " << coords.z  << ":" << std::endl;;
	for (auto & i : children){
		std::cout << i->coords.x << ", " << i->coords.y << ", " << i->coords.z << std::endl;
	}
	std::cout << "finish\n";
}

Node* Node::find_the_closest_node(const Vec3 &point, Node *root_node) {
	auto nearest_node_p = root_node;
	double min_distance = std::numeric_limits<double>::max();
	std::queue<Node*> my_queue;
	
	my_queue.push(root_node);
	
	while (!my_queue.empty()) {
        // front() returns reference, after that we are calling pop,
        // so we need to dereference and take a new pointer to prevent invalid reference
		auto current = &(*my_queue.front());
		my_queue.pop();

		double tmp_dist = Vec3::distance_between_two_vec3(current->coords, point);
		if (tmp_dist < min_distance) {
			min_distance = tmp_dist;
			nearest_node_p = current;
		}
		for (const auto & i : current->children) {
			my_queue.push(i.get());
		}
	}

	return nearest_node_p;
}

void Node::set_as_final() {
	inside_the_goal = true;
}

std::vector<Node*> Node::get_neighbors_in_radius(Node *root,
                                                 const Vec3 &point,
                                                 double radius) {
    std::vector<Node*> neighbors;
    double min_distance = std::numeric_limits<double>::max();
    std::queue<Node*> my_queue;
    auto nearest_node_p = root;

    my_queue.push(root);

    while (!my_queue.empty()) {
        // front() returns reference, after that we are calling pop,
        // so we need to dereference and take a new pointer to prevent invalid reference
        auto current = &(*my_queue.front());
        my_queue.pop();

        double tmp_dist = Vec3::distance_between_two_vec3(current->coords, point);

//        std::cout << radius << " < "<< tmp_dist << " = tmp_dist\n";
        if (tmp_dist < radius){
//            std::cout << tmp_dist << " = tmp_dist\n";
            neighbors.push_back(current);
        }

        for (const auto & i : current->children) {
            my_queue.push(i.get());
        }
    }

    return neighbors;
}

void Node::change_parent(Node *new_parent) {
    for (auto x = this->parent->children.begin(); x < this->parent->children.end(); ++x){
        if (x->get() == this) {
//            std::cout << "here\n";

            new_parent->children.push_back(*x); // (3)

//            std::cout << "About to erase: " << x->get()->cost << "\n";

            this->parent->children.erase(x); // (1)

//            std::cout << "here2\n";

            this->parent = new_parent; // (2)
            break;
        }
    }
}
