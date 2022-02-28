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
#include <algorithm>
#include <string>

#include <ros/ros.h>
#include <mutex>

#include "../math/Vec3.h"
#include "../avoidance/BinarySearchIntersection.h"
#include "../tree_structure/RRT_tree.h"
#include "../path_planning_algorithms/RRTStarAlgorithm.h"
#include "../path_planning_algorithms/RRTAlgorithm.h"

class Drone;

class Trajectory {
public:

    std::vector<std::pair<Vec3, double>> time_points;
    std::vector<Vec3> trajectory_points;
    double dt_between_points = 1;
    double ds_between_points = 1;

    Trajectory() = default;

    Trajectory(const std::vector<Vec3> &path, double dt, double ds);

    void equally_divide_path_in_time(const std::vector<Vec3> &path);

    static void find_trajectories_without_time_collisions(World &global_world,
                                                            std::vector<Drone> &drones,
                                                            bool performance_mode);

private:

    static std::pair<std::vector<Vec3>, int> find_intersects_of_two_trajectories(const Trajectory &priority_traj,
                                                                                 const Trajectory &secondary_traj,
                                                                                 double radius1,
                                                                                 double radius2);

    static unsigned long  last_good_index_of_a_trajectory(World &local_world,
                                                Drone &drone,
                                                unsigned long min_last_good);

};

