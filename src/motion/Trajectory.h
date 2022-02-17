//
// Created by micha on 2/17/2022.
//

#pragma once

#include <mrs_msgs/UavState.h>
#include <mrs_msgs/VelocityReferenceStamped.h>
#include <mrs_msgs/ReferenceStamped.h>
#include <mrs_msgs/TrajectoryReference.h>
#include <mrs_msgs/Reference.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>
#include <string>

#include <ros/ros.h>
#include <mutex>

#include "../math/Vec3.h"
#include "Drone.h"


class Trajectory {
public:

    std::vector<std::pair<Vec3, double>> time_points;
    float dt_between_points;
    float ds_between_points;

    Trajectory(const std::vector<Vec3> &path, const double dt, const double ds){
        dt_between_points = dt;
        ds_between_points = ds;
        time_points = equally_divide_path_in_time(path);
    }

    std::vector<std::pair<Vec3, double>> equally_divide_path_in_time(const std::vector<Vec3> &path){

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


};

