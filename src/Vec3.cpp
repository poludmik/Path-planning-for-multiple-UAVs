//
// Created by misha on 10/10/21.
//

#include "Vec3.h"

Vec3::Vec3(double x, double y, double z) { // vector point // TODO separate ambiguos constructors
	this->x = x;
	this->y = y;
	this->z = z;
}

Vec3 Vec3::operator+(const Vec3 &obj) const {
	Vec3 a;  //create another object
	
	a.x = this->x + obj.x;
	a.y = this->y + obj.y;
	a.z = this->z + obj.z;
	
	return (a); //return object
}

Vec3 Vec3::operator-(const Vec3 &obj) const {
    Vec3 a;  //create another object

    a.x = this->x - obj.x;
    a.y = this->y - obj.y;
    a.z = this->z - obj.z;

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

bool Vec3::line_intersects_sphere(const Vec3 &p1, const Vec3 &p2, const Vec3 &center, double radius) {
    Vec3 p3 = center;
    double a = pow((p1.x - p2.x),2) + pow((p1.y - p2.y),2) + pow((p1.z - p2.z),2);
    double b = 2.0 * ((p2.x-p1.x)*(p1.x-p3.x) + (p2.y-p1.y)*(p1.y-p3.y) + (p2.z-p1.z)*(p1.z-p3.z));
    double c = pow(p3.x, 2) + pow(p3.y, 2) + pow(p3.z, 2) +
            pow(p1.x, 2) + pow(p1.y, 2) + pow(p1.z, 2) -
            2.0 * (p3.x*p1.x + p3.y*p1.y + p3.z*p1.z) - pow(radius, 2);
    double determinant = b * b - 4 * a * c;
    if (determinant < 0.0) { // if > 0 => 2 points of intersection, is == 0 => 1 point.
        return false;
    } else return true;
}

double Vec3::operator|(const Vec3 &V) const {
    return x*V.x + y*V.y + z*V.z;
}

Vec3 Vec3::operator*(double number) const {
    Vec3 a;
    a.x = this->x * number;
    a.y = this->y * number;
    a.z = this->z * number;
    return (a);
}

bool Vec3::DoesLineSegmentIntersectSphere(Vec3 &LinePointStart,
                                          Vec3 &LinePointEnd,
                                          const Vec3 &SphereCenter,
                                          double SphereRadius) {
    const Vec3 LineDiffVect = LinePointEnd - LinePointStart;
//    std::cout << "LineDiffVect: " << LineDiffVect.x << " " << LineDiffVect.y << " " << LineDiffVect.z << "\n";
    const Vec3& copy = LineDiffVect;
    const double lineSegSqrLength = LineDiffVect | copy;

    const Vec3 LineToPointVect = SphereCenter - LinePointStart;
//    std::cout << "LineToPointVect: " << LineToPointVect.x << " " << LineToPointVect.y << " " << LineToPointVect.z << "\n";
    const double dotProduct = LineDiffVect | LineToPointVect;
//    std::cout << "dotProduct: " << dotProduct << "\n";

    double percAlongLine = dotProduct / lineSegSqrLength;
//    std::cout << "percAlongLine: " << percAlongLine << "\n";

    if ( percAlongLine < 0.0l )
    {
        percAlongLine = 0.0l;
    }
    else if ( percAlongLine > 1.0l )
    {
        percAlongLine = 1.0l;
    }

    const Vec3 IntersectionPt = ((LinePointEnd - LinePointStart) * percAlongLine) + LinePointStart;
//    std::cout << "IntersectionPt(closest to center?): " << IntersectionPt.x << " " << IntersectionPt.y << " " << IntersectionPt.z << "\n";
    const Vec3 SpherePtToIntersect = IntersectionPt - SphereCenter;
    const Vec3& copy2 = SpherePtToIntersect;
    const double SqrLengSphereToLine = SpherePtToIntersect | copy2;

    return (SqrLengSphereToLine <= pow(SphereRadius, 2));
}
