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


class Drone {
public:
    std::mutex uav_state_mutex;
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

    explicit Drone(size_t uav_id);

    void odomCallback(mrs_msgs::UavState::ConstPtr const &msg);

};


