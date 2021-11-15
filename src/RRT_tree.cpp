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


std::vector<Vec3> RRT_tree::find_path_to_goal_RRT(World *world_ptr, Vec3& start_point, Vec3& goal_point, double goal_radius, double neighbor_radius) {
	RRT_tree tree(start_point, world_ptr, neighbor_radius);
	
	Vec3 center = (start_point + goal_point) / 2.0;
	
	double dist_to_goal = Vec3::distance_between_two_vec3(start_point, goal_point);

    bool is_inside_an_obstacle = false;

    // if there is a straight line solution to the goal from the start
    for (const auto& obst : world_ptr->obstacles) {
        if (Vec3::DoesLineSegmentIntersectSphere(goal_point, start_point,
                                                 obst.coords, obst.radius)){
            is_inside_an_obstacle = true;
            break;
        }
    }
    if (!is_inside_an_obstacle) {
        auto closest = Node::find_the_closest_node(goal_point, tree.root.get());
        closest->add_child(goal_point);
        return tree.find_way_from_goal_to_root(closest->children.back().get());
    }


	while (true) {

		Vec3 rnd_point = Vec3::random_vec3(center.x - dist_to_goal, center.x + dist_to_goal,
                                           center.y - dist_to_goal, center.y + dist_to_goal,
                                           center.z - dist_to_goal, center.z + dist_to_goal);

        is_inside_an_obstacle = false;

		auto closest = Node::find_the_closest_node(rnd_point, tree.root.get());

        if (Vec3::distance_between_two_vec3(rnd_point, closest->coords) > 2.0) continue;

        for (const auto& obst : world_ptr->obstacles) {
            if (Vec3::DoesLineSegmentIntersectSphere(closest->coords,rnd_point,
                                                     obst.coords, obst.radius)){
                is_inside_an_obstacle = true;
                break;
            }
        }
        if (is_inside_an_obstacle) continue;
        is_inside_an_obstacle = false;

		closest->add_child(rnd_point);

        // if there is a straight line solution to the goal from newly added point
        for (const auto& obst : world_ptr->obstacles) {
            if (Vec3::DoesLineSegmentIntersectSphere(goal_point, rnd_point,
                                                     obst.coords, obst.radius)){
                is_inside_an_obstacle = true;
                break;
            }
        }
        if (is_inside_an_obstacle) {
            if (Vec3::distance_between_two_vec3(rnd_point, goal_point) < goal_radius) {
                return tree.find_way_from_goal_to_root(closest->children.back().get());
            }
        }
        else {
            closest->children.back()->add_child(goal_point);
            return tree.find_way_from_goal_to_root(closest->children.back()->children.back().get());
        }
	}
}


