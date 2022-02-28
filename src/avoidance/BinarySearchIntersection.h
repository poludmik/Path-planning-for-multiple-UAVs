//
// Created by micha on 11/28/2021.
//

#pragma once

#include "../environment_and_objects/Object.h"
#include "../avoidance/AvoidanceAlgorithm.h"

class BinarySearchIntersection : public AvoidanceAlgorithm {

public:

    bool ThereIsIntersectionAlongThePath(const Vec3 &line_start,
                                         const Vec3 &line_end,
                                         double droneRadius,
                                         const Object &obstacle) const override;

};
