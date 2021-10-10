//
// Created by misha on 10/10/21.
//

#pragma once

#include <vector>
#include <random>
#include <iostream>

class Vec3 {
public:
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    
    explicit Vec3() = default;
    
    Vec3(double x, double y, double z);
    
    Vec3 operator +(Vec3 &obj) const;
    
    Vec3(const Vec3& pt);
    
    Vec3 &operator=(const Vec3 &pt) = default;
    
    static Vec3 random_vec3(double range_a, double range_b);
};
