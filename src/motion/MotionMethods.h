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

class MotionMethods {
public:

    static void go_to_point_proportional(Drone &drone, const Vec3& point);

    static void go_to_the_point(Drone &drone, const Vec3& point){

        drone.cmd_goto.reference.position.x = point.x;
        drone.cmd_goto.reference.position.y = point.y;
        drone.cmd_goto.reference.position.z = point.z;
        drone.cmd_goto.header.frame_id = drone.global_frame_id;
        drone.cmd_goto.reference.heading = 0.0;
        drone.cmd_goto.header.stamp = ros::Time::now();
        drone.goto_pub.publish(drone.cmd_goto);

        ros::Rate rate(10);
        rate.sleep();
        ros::spinOnce();
    }

};


