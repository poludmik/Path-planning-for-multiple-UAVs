//
// Created by misha on 10/10/21.
//

#include "Vec3.h"

Vec3::Vec3(double x, double y, double z, bool c) { // vector point // TODO separate ambiguos constructors
	coords.reserve(3);
	coords[0] = x;
	coords[1] = y;
	coords[2] = z;
}

Vec3::Vec3(bool random, double range_a, double range_b) { // random or zero vector point
	coords.reserve(3);
	if (random) {
		std::random_device rd;  // Will be used to obtain a seed for the random number engine
		std::mt19937 generator(rd()); // Standard mersenne_twister_engine seeded with rd()
		std::uniform_real_distribution<> distribution(range_a, range_b);
		
		for (int i = 0; i < 3; ++i) {
			coords[i] = distribution(generator);
		}
	} else {
		coords[0] = 0.0;
		coords[1] = 0.0;
		coords[2] = 0.0;
	}
}

Vec3 Vec3::operator+(Vec3 &obj) {
	Vec3 x;  //create another object
	
	for (int i = 0; i < 3; ++i) {
		x.coords[i] = this->coords[i] + obj.coords[i];
	}
	return (x); //return object
}

Vec3::Vec3(const Vec3 &pt) { // copy constructor //TODO nachren on nuzen
	this->coords[0] = pt.coords[0];
	this->coords[1] = pt.coords[1];
	this->coords[2] = pt.coords[2];
}

Vec3 &Vec3::operator=(const Vec3 &pt) {
	coords[0] = pt.coords[0];
	coords[1] = pt.coords[1];
	coords[2] = pt.coords[2];
	return *this;
}
