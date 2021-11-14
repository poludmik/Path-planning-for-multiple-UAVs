//
// Created by micha on 11/14/2021.
//

#pragma once

#include "Vec3.h"
#include "Node.h"
#include "World.h"


class Algorithm {
public:
    virtual std::vector<Vec3> find_path_according_to_alg(const World *world_ptr,
                                                         Node *root,
                                                         const Vec3 &start_point,
                                                         const Vec3 &goal_point,
                                                         double goal_radius,
                                                         double neighbor_radius) const = 0;

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

};

