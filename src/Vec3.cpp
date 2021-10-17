//
// Created by misha on 10/10/21.
//

#include "Vec3.h"

Vec3::Vec3(double x, double y, double z) { // vector point // TODO separate ambiguos constructors
	this->x = x;
	this->y = y;
	this->z = z;
}

Vec3 Vec3::operator+(Vec3 &obj) const {
	Vec3 a;  //create another object
	
	a.x = this->x + obj.x;
	a.y = this->y + obj.y;
	a.z = this->z + obj.z;
	
	return (a); //return object
}

Vec3 Vec3::operator/(double number) const {
	return {x/number, y/number, z/number};
}

Vec3::Vec3(const Vec3 &pt) { // copy constructor
	this->x = pt.x;
	this->y = pt.y;
	this->z = pt.z;
}

Vec3 Vec3::random_vec3(double range_ax, double range_bx,
                       double range_ay, double range_by,
                       double range_az, double range_bz) {
	
	std::random_device rd;  // Will be used to obtain a seed for the random number engine
	std::mt19937 generator(rd()); // Standard mersenne_twister_engine seeded with rd()
	std::uniform_real_distribution<> distributionX(range_ax, range_bx);
	std::uniform_real_distribution<> distributionY(range_ay, range_by);
	std::uniform_real_distribution<> distributionZ(range_az, range_bz);
	
	return {distributionX(generator), distributionY(generator), distributionZ(generator)};
}

double Vec3::distance_between_two_vec3(const Vec3 &pt1, const Vec3 &pt2) {
	return sqrt(pow(pt1.x - pt2.x, 2) +
	            pow(pt1.y - pt2.y, 2) +
	            pow(pt1.z - pt2.z, 2));
}

