//
// Created by micha on 2/28/2022.
//

#pragma once

#include "Object.h"

class Cylinder : public Object {

public:
    double height;

    Cylinder(double radius, const Vec3 &givenCoords, double height) : Object(radius, givenCoords), height(height) {
        coords.z += height / 2;
    }

};

