//
// Created by micha on 2/17/2022.
//

#include "Trajectory.h"

std::vector<std::pair<Vec3, double>> Trajectory::equally_divide_path_in_time(const std::vector<Vec3> &path) const {

    std::vector<std::pair<Vec3, double>> trajectory;
    std::pair<Vec3, double> point_in_time;

    double global_time_shift = 0.0;

    for (uint i = 0; i < path.size() - 1; ++i){
        Vec3 current_point = path[i];
        Vec3 next_point = path[i + 1];
        Vec3 direction = next_point - current_point;

        double distance = Vec3::distance_between_two_vec3(current_point, next_point);

        point_in_time.first = current_point;
        point_in_time.second = global_time_shift;
        trajectory.push_back(point_in_time);

        double current_dist_shift = ds_between_points;

        while (current_dist_shift < distance) {
            Vec3 between_point(current_point.x + direction.x * current_dist_shift / distance,
                               current_point.y + direction.y * current_dist_shift / distance,
                               current_point.z + direction.z * current_dist_shift / distance);


            if (Vec3::distance_between_two_vec3(between_point, next_point) > ds_between_points / 3) {

                global_time_shift += dt_between_points;
                current_dist_shift += ds_between_points;

                point_in_time.first = between_point;
                point_in_time.second = global_time_shift;

                trajectory.push_back(point_in_time);
            } else {
                break;
            }
        }

        global_time_shift += dt_between_points;

    }

    point_in_time.first = path.back();
    point_in_time.second = global_time_shift;
    trajectory.push_back(point_in_time);

    return trajectory;
}

Trajectory::Trajectory(const std::vector<Vec3> &path, const double dt, const double ds) {
    dt_between_points = dt;
    ds_between_points = ds;
    time_points = equally_divide_path_in_time(path);
}

std::vector<Vec3> Trajectory::find_intersects_of_two_trajectories(const Trajectory &traj1,
                                                                  const Trajectory &traj2,
                                                                  double radius1,
                                                                  double radius2) {

    std::vector<Vec3> intersection_points;

    double time_epsilon = 0.001;

    for (const auto &first_time_point : traj1.time_points) {

        for (const auto &second_time_point : traj2.time_points) {
            if (first_time_point.second < second_time_point.second + time_epsilon &&
                first_time_point.second > second_time_point.second - time_epsilon){

                if (AvoidanceAlgorithm::DoesSphereIntersectSphere(first_time_point.first,
                                                                  second_time_point.first,
                                                                  radius1,
                                                                  radius2))
                    intersection_points.push_back(first_time_point.first);
            }
        }

    }

    return intersection_points;
}
