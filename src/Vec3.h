//
// Created by misha on 10/10/21.
//

#pragma once

#include <vector>
#include <random>
#include <iostream>
#include <cmath>

class Vec3 {
public:
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    
    explicit Vec3() = default;
    
    Vec3(double x, double y, double z);
    
    Vec3 operator +(Vec3 &obj) const;
    
    Vec3 operator/(double number) const;
    
    Vec3(const Vec3& pt);
    
    Vec3 &operator=(const Vec3 &pt) = default;
    
    static Vec3 random_vec3(double range_ax, double range_bx,
                            double range_ay, double range_by,
                            double range_az, double range_bz);
    
    static double distance_between_two_vec3(const Vec3 &pt1, const Vec3 &pt2);
};
