//
// Created by misha on 10/10/21.
//

#pragma once

#include <vector>
#include <random>
#include <iostream>

class Vec3 {
public:
    std::vector<double> coords;
    
    Vec3(double x, double y, double z, bool c);
    
    explicit Vec3(bool random=false, double range_a=-1.0, double range_b=1.0);
    
    Vec3 operator +(Vec3 &obj);
    
    Vec3(const Vec3& pt);
    
    Vec3& operator = (const Vec3 &pt);
};
