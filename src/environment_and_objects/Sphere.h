//
// Created by micha on 2/28/2022.
//

#pragma once
#include "Object.h"

class Sphere : public Object {

public:
    Sphere(double radius, const Vec3 &givenCoords) : Object(radius, givenCoords) {}

};
