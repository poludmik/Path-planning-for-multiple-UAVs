//
// Created by misha on 10/10/21.
//

#pragma once

#include <vector>
#include <string>
#include <iostream>
#include <cmath>
#include "../math/Vec3.h"
#include "World.h"


class Object{
public:
    
    std::string name;
    double radius;
    Vec3 coords;
    bool is_goal = false;
    bool is_start = false;
    bool is_obstacle = false;
    
    void print_out_info() const;
    
    void set_as_a_goal();
    
    void set_as_a_start();

    void set_as_an_obstacle();
    
    void finish_being_goal();
    
    static bool are_intersecting(const Object &obj1, const Object &obj2);
    
    Object(const World* pBigWorld, const std::string &name, double radius,
           const Vec3 &given_coords);
    
    Object(const World* pBigWorld, double radius, const Vec3 &given_coords);
    
    Object (const Object& obj);
    
    ~Object();

private:
    const World* pBigWorld;
    
    static double distance_between_two_centers(const Object &obj1, const Object &obj2);
};