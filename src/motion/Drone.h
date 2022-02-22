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

#include "../motion/Trajectory.h"

class Drone {
public:

    size_t uav_id;

    Vec3 start_point;

    Vec3 goal_point;

    double goal_radius;

    double drone_radius;

    std::vector<Vec3> found_path;

    Trajectory trajectory;

    ros::NodeHandle n;
    bool ready = false;
    mrs_msgs::UavState::ConstPtr uav_state;
    mrs_msgs::VelocityReferenceStamped cmd_vel;
    mrs_msgs::ReferenceStamped cmd_goto;
    mrs_msgs::TrajectoryReference cmd_trajectory;
    ros::Subscriber odom_sub;
    ros::Publisher  vel_pub;
    ros::Publisher  goto_pub;
    ros::Publisher  trajectory_pub;
    std::string local_frame_id;
    std::string global_frame_id;


    Drone(size_t uav_id, const Vec3 &start_point, const Vec3 &goal_point, double goal_radius, double drone_radius);

    void odomCallback(mrs_msgs::UavState::ConstPtr const &msg);

};
