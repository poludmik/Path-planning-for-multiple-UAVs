//
// Created by micha on 11/28/2021.
//

#pragma once

#include "../math/Vec3.h"
#include "../tree_structure/Node.h"
#include "../environment_and_objects/World.h"


class AvoidanceAlgorithm {
public:

    virtual bool ThereIsIntersectionAlongThePath(const Vec3 &line_start,
                                                 const Vec3 &line_end,
                                                 double droneRadius,
                                                 const Object &obstacle) const  = 0;

    static bool DoesSphereIntersectSphere(const Vec3 &firstSphereCenter,
                                          const Vec3 &secondSphereCenter,
                                          double firstSphereRadius,
                                          double secondSphereRadius) {
        return Vec3::distance_between_two_vec3(firstSphereCenter, secondSphereCenter) <= firstSphereRadius + secondSphereRadius;
    }

    static bool line_intersects_sphere(const Vec3 &p1, const Vec3 &p2, const Vec3 &center, double radius) {
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

};
