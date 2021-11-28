//
// Created by micha on 11/28/2021.
//

#include "BinarySearchIntersection.h"


bool BinarySearchIntersection::ThereIsIntersectionAlongThePath(const Vec3 &line_start,
                                                               const Vec3 &line_end,
                                                               const Vec3 &obstacleCoords,
                                                               double droneRadius,
                                                               double obstacleRadius) const {

    double current_step = Vec3::distance_between_two_vec3(line_end, line_start);
    double iter_num = 8;
    double minimal_step = current_step / (pow(2, iter_num) + 1);

    std::vector<Vec3> end_points;
    end_points.push_back(line_start);
    end_points.push_back(line_end);
    Vec3 middle_point;

    while (current_step > minimal_step) {
        middle_point = Vec3((end_points[0].x + end_points[1].x) / 2,
                            (end_points[0].y + end_points[1].y) / 2,
                            (end_points[0].z + end_points[1].z) / 2);
        current_step = current_step / 2;
//        std::cout << middle_point.x << " " << middle_point.y << " " << middle_point.z << "\n";

        if (DoesSphereIntersectSphere(middle_point, obstacleCoords, droneRadius, obstacleRadius)) {
            return true;
        }

        end_points.push_back(middle_point);
        auto max_iter = end_points.begin();
        double max_distance = std::numeric_limits<double>::min();
        for (auto i = end_points.begin(); i < end_points.end(); ++i) {
            if (Vec3::distance_between_two_vec3(obstacleCoords, *i) > max_distance) {
                max_distance = Vec3::distance_between_two_vec3(obstacleCoords, *i);
                max_iter = i;
            }
        }
        end_points.erase(max_iter);
    }
    return false;
}
