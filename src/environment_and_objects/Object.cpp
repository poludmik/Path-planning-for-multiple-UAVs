//
// Created by misha on 10/10/21.
//

#include "Object.h"

void Object::print_out_info() const {
	std::cout << "name: " << type << std::endl;
    std::cout << "IsGoal: " << is_goal << std::endl;
    std::cout << "IsStart: " << is_start << std::endl;
    std::cout << "Radius: " << radius << std::endl;
    std::cout << "Height: " << height << std::endl;
	std::cout << "x: " << coords.x << std::endl;
	std::cout << "y: " << coords.y << std::endl;
	std::cout << "z: " << coords.z << std::endl;
	std::cout << "\n";
}

Object::Object(const World *pBigWorld, const std::string &type, double radius,
               const Vec3 &given_coords) {
	this->pBigWorld = pBigWorld;
    if (type != "cylinder" and type != "sphere"){
        std::cout << "Unknown object type: " << type << ". Making a sphere." << std::endl;
        this->type = "sphere";
        this->coords = given_coords;
        this->height = 0.0;
    } else if (type == "cylinder"){
        std::cout << "Assuming height = 2\n";
        this->height = 2.0;
        this->type = type;
        this->coords = given_coords;
        this->coords.z = this->coords.z + height / 2;
    } else if (type == "sphere"){
        this->type = type;
        this->coords = given_coords;
        this->height = 0.0;
    }

	this->radius = radius;

//	std::cout << "Object '" << name << "' has been created.\n" << std::endl;
}

Object::Object(const World *pBigWorld, double radius, const Vec3 &given_coords) {
	this->pBigWorld = pBigWorld;
	this->type = "sphere";
	this->radius = radius;
    this->height = 0;
	this->coords = given_coords;
//	std::cout << "Object '" << "obj" << "' has been created.\n" << std::endl;
}

Object::Object(const Object &obj) { // copy constructor
	pBigWorld = obj.pBigWorld;
    type = obj.type;
	radius = obj.radius;
	coords = obj.coords;
    height = obj.height;
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

Object::Object(const World *pBigWorld, const std::string &type, double radius, const Vec3 &given_coords,
               const double height) {
    this->pBigWorld = pBigWorld;
    if (type != "cylinder" and type != "sphere"){
        std::cout << "Unknown object type: " << type << ". Making a sphere." << std::endl;
        this->type = "sphere";
        this->coords = given_coords;
        this->height = 0.0;
    } else if (type == "cylinder"){
        this->height = height;
        this->type = type;
        this->coords = given_coords;
        this->coords.z = this->coords.z + height / 2;
    } else if (type == "sphere"){
        this->type = type;
        this->coords = given_coords;
        this->height = 0.0;
    }

    this->radius = radius;
//	std::cout << "Object '" << name << "' has been created.\n" << std::endl;

}


