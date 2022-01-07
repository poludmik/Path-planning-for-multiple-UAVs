//
// Created by micha on 11/13/2021.
//

#include "math/Vec3.h"
#include <ros/ros.h>
#include <mrs_msgs/UavState.h>
#include <mrs_msgs/VelocityReferenceStamped.h>


#pragma once

class VelocityControllerP {
public:


    static void fly_along_given_path(const std::vector<Vec3>& path, const mrs_msgs::UavState::ConstPtr& uav_state)
    {
        for (const auto& point : path) {

            printf("%lf %lf %lf\n", point.x, point.y, point.z);
        }
    }

    static void go_to_point(const Vec3& point, mrs_msgs::UavState::ConstPtr& uav_state,
                            mrs_msgs::VelocityReferenceStamped cmd,
                            const ros::Publisher& vel_pub)
                            {
        ros::Rate rate(10);


//        ROS_INFO("I heard: [%f %f %f]", uav_state->pose.position.x, uav_state->pose.position.y, uav_state->pose.position.z);
        Vec3 e = point - Vec3(uav_state->pose.position.x, uav_state->pose.position.y, uav_state->pose.position.z);

        while (e.norm() > 0.2) {

            std::cout << uav_state->pose.position.x << "\n";
            cmd.reference.velocity.x = e.x * 0.1;
            cmd.reference.velocity.y = e.y * 0.1;
            cmd.reference.velocity.z = e.z * 0.1;

            vel_pub.publish(cmd);

            e = point - Vec3(uav_state->pose.position.x, uav_state->pose.position.y, uav_state->pose.position.z);

            ros::spinOnce();
            rate.sleep();
        }

    }

};

