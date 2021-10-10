//
// Created by misha on 10/10/21.
//

#include "Object.h"

void Object::print_out_info() const {
	std::cout << "name: " << name << std::endl;
	std::cout << "x: " << coords.x << std::endl; // TODO, ask vanya, tupo vyglyadit
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
	std::cout << "Object '" << name << "' has been created.\n" << std::endl;
}

Object::Object(const World *pBigWorld, double radius, const Vec3 &given_coords) {
	this->pBigWorld = pBigWorld;
	this->name = "obj";
	this->radius = radius;
	this->coords = given_coords;
	std::cout << "Object '" << "obj" << "' has been created.\n" << std::endl;
}

Object::Object(const Object &obj) { // copy constructor
	pBigWorld = obj.pBigWorld;
	name = obj.name;
	radius = obj.radius;
	coords = obj.coords;
}

Object::~Object() {
	std::cout << "Object '" << name << "' has been destroyed." << std::endl;
}
