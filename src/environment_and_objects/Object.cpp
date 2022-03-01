//
// Created by misha on 10/10/21.
//

#include "Object.h"

void Object::set_as_a_start() { is_a_start = true;}

void Object::set_as_a_goal() { is_a_goal = true; }

Object::Object(double radius, const Vec3 &given_coords) : radius(radius), coords(given_coords) {}
