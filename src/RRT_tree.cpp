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

void RRT_tree::write_tree_structure_to_json_file(Node *root,
                                                 const std::string& tree_name,
                                                 const std::string& filename,
                                                 const std::vector<Vec3> &path,
                                                 const Vec3 &goal,
                                                 const double goal_radius,
                                                 const std::vector<Object> &obstacles) {
    std::queue<Node *> my_queue;

    my_queue.push(root);

    nlohmann::json j_structure;

    std::ofstream o(filename);

    size_t idx = 1;
    while (!my_queue.empty()) {
        // front() returns reference, after that we are calling pop,
        // so we need to dereference and take a new pointer to prevent invalid reference
        auto current = &(*my_queue.front());
        my_queue.pop();

        for (const auto &i: current->children) {

//            std::cout << idx << "\n";
            std::vector<double> line_sector;

            line_sector.push_back(current->coords.x);
            line_sector.push_back(current->coords.y);
            line_sector.push_back(current->coords.z);

            line_sector.push_back(i->coords.x);
            line_sector.push_back(i->coords.y);
            line_sector.push_back(i->coords.z);

            j_structure["tree" + std::to_string(idx)] = line_sector;

            line_sector.clear();

            my_queue.push(i.get());
            ++idx;
        }
    }
    j_structure["tree_size"] = idx;

    j_structure["start"] = {root->coords.x, root->coords.y, root->coords.z};
    j_structure["goal"] = {goal.x, goal.y, goal.z};
    idx = 1;
    for (const auto &i : path){
        std::vector<double> path_point;

        path_point.push_back(i.x);
        path_point.push_back(i.y);
        path_point.push_back(i.z);

        j_structure["path" + std::to_string(idx)] = path_point;
        path_point.clear();
        ++idx;
    }
    j_structure["path_size"] = idx;

    j_structure["goal_radius"] = goal_radius;

    // Write obstacles
    idx = 1;
    for (const auto &x : obstacles){
        std::vector<double> one_obstacle;
        one_obstacle.push_back(x.radius);
        one_obstacle.push_back(x.coords.x);
        one_obstacle.push_back(x.coords.y);
        one_obstacle.push_back(x.coords.z);
        j_structure["obstacle" + std::to_string(idx)] = one_obstacle;
        one_obstacle.clear();
        ++idx;
    }
    j_structure["obstacles_number"] = idx;

    j_structure["graph_name"] = tree_name;

    o << std::setw(8) << j_structure << std::endl;
}
