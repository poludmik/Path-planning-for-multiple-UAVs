//
// Created by micha on 2/9/2022.
//

#include "MotionMethods.h"

void MotionMethods::go_to_point_proportional(Drone &drone, const Vec3& point){

    ros::Rate rate(10);
    mrs_msgs::UavState::ConstPtr cur_uav_state;
    ros::spinOnce();
    rate.sleep();


    while (ros::ok()) {

        if (drone.ready) {

            cur_uav_state = drone.uav_state;

            Vec3 curr_position = Vec3(cur_uav_state->pose.position.x,
                                      cur_uav_state->pose.position.y,
                                      cur_uav_state->pose.position.z);

            Vec3 error = point - curr_position;

            if (error.norm() > 0.1) {
                drone.cmd_vel.reference.velocity.x = error.x * 0.3;
                drone.cmd_vel.reference.velocity.y = error.y * 0.3;
                drone.cmd_vel.reference.velocity.z = error.z * 0.3;

                drone.vel_pub.publish(drone.cmd_vel);
            } else {
                break;
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
}
