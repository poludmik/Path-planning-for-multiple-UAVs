//
// Created by misha on 10/10/21.
//

#pragma once

#include <vector>
#include <string>
#include <iostream>
#include <cmath>
#include "../math/Vec3.h"

class Object{
public:
    double radius;
    Vec3 coords;
    
    void set_as_a_goal();
    
    void set_as_a_start();

    bool is_goal() const {
        return is_a_goal;
    }

    bool is_start() const {
        return is_a_start;
    }

    Object(double radius, const Vec3 &given_coords);
    
    virtual ~Object() {}
private:
    bool is_a_goal = false;
    bool is_a_start = false;
};
