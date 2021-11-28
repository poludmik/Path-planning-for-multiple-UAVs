//
// Created by micha on 11/28/2021.
//

#pragma once

#include "Object.h"
#include "AvoidanceAlgorithm.h"

class LinearAlgebraIntersection : public AvoidanceAlgorithm {

public:

    bool ThereIsIntersectionAlongThePath(const Vec3 &LinePointStart,
                                         const Vec3 &LinePointEnd,
                                         const Vec3 &obstacleCoords,
                                         double droneRadius,
                                         double obstacleRadius) const override;

};
