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
#include "../avoidance/AvoidanceAlgorithm.h"


class Trajectory {
public:

    std::vector<std::pair<Vec3, double>> time_points;
    double dt_between_points = 1;
    double ds_between_points = 1;

    Trajectory() = default;

    Trajectory(const std::vector<Vec3> &path, double dt, double ds);

    std::vector<std::pair<Vec3, double>> equally_divide_path_in_time(const std::vector<Vec3> &path) const;

    static std::vector<Vec3> find_intersects_of_two_trajectories(const Trajectory &traj1,
                                                                 const Trajectory &traj2,
                                                                 double radius1,
                                                                 double radius2);


};

