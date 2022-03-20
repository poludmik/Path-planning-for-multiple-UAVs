//
// Created by micha on 2/9/2022.
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
#include "../math/Orientation.h"

class MotionMethods {
public:

    static void go_to_point_proportional(Drone &drone, const Vec3& point);

    static void go_to_the_point(Drone &drone, const Vec3& point);

    static void go_through_a_trajectory(Drone &drone, const std::vector<Vec3> &path, double dt);

};

