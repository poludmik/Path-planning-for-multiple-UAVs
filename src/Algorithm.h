//
// Created by micha on 11/14/2021.
//

#pragma once

#include "Vec3.h"
#include "Node.h"
#include "World.h"
#include "AvoidanceAlgorithm.h"


class Algorithm {
public:
    virtual std::vector<Vec3> find_path_according_to_alg(const World *world_ptr,
                                                         const AvoidanceAlgorithm &avoid_alg,
                                                         Node *root,
                                                         const Vec3 &start_point,
                                                         const Vec3 &goal_point,
                                                         double goal_radius,
                                                         double neighbor_radius,
                                                         double droneRadius) const = 0;

    static std::vector<Vec3> find_way_from_goal_to_root(const Node *last_goal_p) {
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

    static std::vector<std::shared_ptr<Node>> get_neighbors_in_radius(const Node *root,
                                                                      const Vec3& point,
                                                                      double radius) {
        std::vector<std::shared_ptr<Node>> neighbors;
        std::vector<std::shared_ptr<Node>> stack;

        for (auto &x:root->children){
            stack.push_back(x);
        }

        std::shared_ptr<Node> current;

        while (!stack.empty()) {

            current = stack.back();

            if (Vec3::distance_between_two_vec3(current->coords, point) <= radius) {
                neighbors.emplace_back(current);
            }

            stack.pop_back();

            for (const auto &x:current->children){
                stack.push_back(x);
            }
        }

        return neighbors;
    }

};

