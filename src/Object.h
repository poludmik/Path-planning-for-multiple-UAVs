//
// Created by misha on 10/10/21.
//

#pragma once

#include <vector>
#include <string>
#include <iostream>
#include "Vec3.h"
#include "World.h"


class Object{
public:
    
    std::string name;
    double radius;
    Vec3 coords;
    bool is_goal = false;
    
    void print_out_info() const;
    
    void set_as_a_goal(){ is_goal = true; }
    
    void finish_being_goal(){ is_goal = false; }
    
    Object(const World* pBigWorld, const std::string &name, double radius,
           const Vec3 &given_coords);
    
    Object(const World* pBigWorld, double radius, const Vec3 &given_coords);
    
    Object (const Object& obj);
    
    ~Object();

private:
    const World* pBigWorld;
};