//
// Created by micha on 11/28/2021.
//

#include "../avoidance/BinarySearchIntersection.h"


bool BinarySearchIntersection::ThereIsIntersectionAlongThePath(const Vec3 &line_start,
                                                               const Vec3 &line_end,
                                                               const Vec3 &obstacleCoords,
                                                               double droneRadius,
                                                               double obstacleRadius,
                                                               double obstacleHeight) const {

    double current_step = Vec3::distance_between_two_vec3(line_end, line_start);
    double iter_num = 8;
    double minimal_step = current_step / (pow(2, iter_num) + 1);

    std::vector<Vec3> end_points;
    end_points.push_back(line_start);
    end_points.push_back(line_end);
    Vec3 middle_point;
    Vec3 cylinder_top_point(obstacleCoords.x, obstacleCoords.y, obstacleCoords.z + obstacleHeight/2.0);
    Vec3 cylinder_bottom_point(obstacleCoords.x, obstacleCoords.y, obstacleCoords.z - obstacleHeight/2.0);

    auto find_closest_point_on_a_cylinder_axis = [&](const Vec3 &point, const Vec3 &A, const Vec3 &B) {
        Vec3 a_to_p = point - A;    // vector A->P
        Vec3 a_to_b = B - A;        // vector A->B
        const Vec3 &copy = a_to_b;
        double atb2 = a_to_b | copy;
        double atp_dot_atb = a_to_p | a_to_b;
        double t = atp_dot_atb / atb2;
        if (t > 1)
            return B;
        else if (t < 0)
            return A;
        else
            return Vec3(A.x + a_to_b.x * t, A.y + a_to_b.y * t, A.z + a_to_b.z * t);
    };

    Vec3 closest_point;

    while (current_step > minimal_step) {
        middle_point = Vec3((end_points[0].x + end_points[1].x) / 2,
                            (end_points[0].y + end_points[1].y) / 2,
                            (end_points[0].z + end_points[1].z) / 2);
        current_step = current_step / 2;

        if (obstacleHeight > 0.0) // cylinder
            closest_point = find_closest_point_on_a_cylinder_axis(middle_point, cylinder_bottom_point, cylinder_top_point);
        else
            closest_point = obstacleCoords;


        if (DoesSphereIntersectSphere(middle_point, closest_point, droneRadius, obstacleRadius)) {
            /*
                   The cylinder avoidance is done by finding the closest point on a cylinders axis and
               doing a sphere/sphere intersection check in that point. But on the top and bottom of the cylinder
               I introduced this rather big condition to check "rectangular" shape instead of a sphere.
               And if you are shocked by this enormous and extremely understandable IF statement, please know that
               I also find it quiet humongous. Also, I am writing this pretty tired after midnight, and I don't really
               want to split this condition into smaller ones, so let's just forget about it. It works amazingly.
           */
            if (obstacleHeight > 0 and !(((line_start.z > cylinder_top_point.z + droneRadius and line_end.z > cylinder_top_point.z + droneRadius)
            and (closest_point == cylinder_top_point and middle_point.z - droneRadius/2.0 > cylinder_top_point.z)) or
            ((line_start.z < cylinder_bottom_point.z - droneRadius and line_end.z < cylinder_bottom_point.z - droneRadius)
            and (closest_point == cylinder_bottom_point and middle_point.z + droneRadius < cylinder_bottom_point.z))))
                return true;
            else if (obstacleHeight <= 0) // if obstacle is a sphere
                return true;
        }

        end_points.push_back(middle_point);
        auto max_iter = end_points.begin();
        double max_distance = std::numeric_limits<double>::min();
        for (auto i = end_points.begin(); i < end_points.end(); ++i) {
            if (Vec3::distance_between_two_vec3(closest_point, *i) > max_distance) {
                max_distance = Vec3::distance_between_two_vec3(closest_point, *i);
                max_iter = i;
            }
        }
        end_points.erase(max_iter);
    }
    return false;
}
