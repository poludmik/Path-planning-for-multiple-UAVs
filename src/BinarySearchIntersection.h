//
// Created by micha on 11/28/2021.
//

#pragma once

#include "Object.h"
#include "AvoidanceAlgorithm.h"

class BinarySearchIntersection : public AvoidanceAlgorithm {

public:

    bool ThereIsIntersectionAlongThePath(const Vec3 &line_start,
                                         const Vec3 &line_end,
                                         const Vec3 &obstacleCoords,
                                         double droneRadius,
                                         double obstacleRadius) const override;

};
