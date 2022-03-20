//
// Created by micha on 3/20/2022.
//

#include "Orientation.h"

double Orientation::get_heading_in_rad_from_quaternion(double x, double y, double z, double w) {
    // Get drones heading/yaw angle.
    return atan2(2.0f * (w * z + x * y), w * w + x * x - y * y - z * z);
}

Vec3 Orientation::rotate_vector_around_z(const Vec3 &point, const double angle_rad) {
    // Rotational matrix around z-axis.
    return Vec3(cos(angle_rad) * point.x - sin(angle_rad) * point.y, sin(angle_rad) * point.x + cos(angle_rad) * point.y, point.z);
}
