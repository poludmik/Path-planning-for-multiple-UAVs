//
// Created by misha on 10/10/21.
//

#include "Object.h"

void Object::print_out_info() const {
	std::cout << "name: " << type << std::endl;
    std::cout << "IsGoal: " << is_a_goal << std::endl;
    std::cout << "IsStart: " << is_a_start << std::endl;
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
    is_a_goal = obj.is_a_goal;
    is_a_start = obj.is_a_start;
}

Object::~Object() {
//	std::cout << "Object '" << name << "' has been destroyed." << std::endl;
}

double Object::distance_between_two_centers(const Object &obj1, const Object &obj2) {
	return sqrt(pow(obj1.coords.x - obj2.coords.x, 2) +
		            pow(obj1.coords.y - obj2.coords.y, 2) +
			    pow(obj1.coords.z - obj2.coords.z, 2));
}

void Object::set_as_a_start() { is_a_start = true;}

void Object::set_as_a_goal() { is_a_goal = true; }

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


