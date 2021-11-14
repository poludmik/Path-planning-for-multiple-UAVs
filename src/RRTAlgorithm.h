//
// Created by micha on 11/14/2021.
//

#pragma once

#include "Algorithm.h"
#include "Object.h"

class RRTAlgorithm : public Algorithm {
    std::vector<Vec3> find_path_according_to_alg(const World *world_ptr,
                                                 Node *root,
                                                 const Vec3 &start_point,
                                                 const Vec3 &goal_point,
                                                 double goal_radius,
                                                 double neighbor_radius) const override;
};
