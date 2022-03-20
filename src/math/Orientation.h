//
// Created by micha on 3/20/2022.
//

#pragma once
#include <cmath>
#include <iostream>
#include "Vec3.h"

class Orientation {

public:

    static double get_heading_in_rad_from_quaternion(double x, double y, double z, double w);

    static Vec3 rotate_vector_around_z(const Vec3 &point, double angle_rad);

};


