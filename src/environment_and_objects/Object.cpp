//
// Created by misha on 10/10/21.
//

#include "Object.h"

void Object::print_out_info() const {
	std::cout << "name: " << name << std::endl;
    std::cout << "IsGoal: " << is_goal << std::endl;
    std::cout << "IsStart: " << is_start << std::endl;
	std::cout << "x: " << coords.x << std::endl;
	std::cout << "y: " << coords.y << std::endl;
	std::cout << "z: " << coords.z << std::endl;
	std::cout << "\n";
}

Object::Object(const World *pBigWorld, const std::string &name, double radius,
               const Vec3 &given_coords) {
	this->pBigWorld = pBigWorld;
	this->name = name;
	this->radius = radius;
	this->coords = given_coords;
//	std::cout << "Object '" << name << "' has been created.\n" << std::endl;
}

Object::Object(const World *pBigWorld, double radius, const Vec3 &given_coords) {
	this->pBigWorld = pBigWorld;
	this->name = "obj";
	this->radius = radius;
	this->coords = given_coords;
//	std::cout << "Object '" << "obj" << "' has been created.\n" << std::endl;
}

Object::Object(const Object &obj) { // copy constructor
	pBigWorld = obj.pBigWorld;
	name = obj.name;
	radius = obj.radius;
	coords = obj.coords;
    is_goal = obj.is_goal;
    is_start = obj.is_start;
    is_obstacle = obj.is_obstacle;
}

Object::~Object() {
//	std::cout << "Object '" << name << "' has been destroyed." << std::endl;
}

bool Object::are_intersecting(const Object &obj1, const Object &obj2) {
	if (Object::distance_between_two_centers(obj1, obj2) <= (obj1.radius/2 + obj2.radius/2))
	{ return true; } else { return false; }
}

double Object::distance_between_two_centers(const Object &obj1, const Object &obj2) {
	return sqrt(pow(obj1.coords.x - obj2.coords.x, 2) +
		            pow(obj1.coords.y - obj2.coords.y, 2) +
			    pow(obj1.coords.z - obj2.coords.z, 2));
}

void Object::set_as_a_start() { is_start = true;}

void Object::set_as_a_goal() { is_goal = true; }

void Object::set_as_an_obstacle() { is_obstacle = true; }

void Object::finish_being_goal() { is_goal = false; }


