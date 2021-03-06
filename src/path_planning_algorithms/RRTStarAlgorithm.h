//
// Created by micha on 11/14/2021.
//


#pragma once

#include "Algorithm.h"

class RRTStarAlgorithm : public Algorithm {
    std::vector<Vec3> find_path_according_to_alg(const World *world_ptr,
                                                 const AvoidanceAlgorithm &avoid_alg,
                                                 Node *root,
                                                 const Vec3 &start_point,
                                                 const Vec3 &goal_point,
                                                 double goal_radius,
                                                 double neighbor_radius,
                                                 double droneRadius) const override;

};
