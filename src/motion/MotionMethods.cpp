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

void MotionMethods::go_to_the_point(Drone &drone, const Vec3 &point) {

    drone.cmd_goto.reference.position.x = point.x;
    drone.cmd_goto.reference.position.y = point.y;
    drone.cmd_goto.reference.position.z = point.z;
    drone.cmd_goto.header.frame_id = drone.local_frame_id; // point is assumed to be in a local coordinate system
    drone.cmd_goto.reference.heading = 0.0;
    drone.cmd_goto.header.stamp = ros::Time::now();
    drone.goto_pub.publish(drone.cmd_goto);

    ros::Rate rate(10);
    rate.sleep();
    ros::spinOnce();
}

void MotionMethods::go_through_a_trajectory(Drone &drone, const std::vector<Vec3> &path, const double dt) {
    std::vector<mrs_msgs::Reference> array;

    for (const auto &point : path) {
        mrs_msgs::Reference reference;
        reference.position.x = point.x;
        reference.position.y = point.y;
        reference.position.z = point.z;

        array.push_back(reference);
    }

    drone.cmd_trajectory.points = array;
    drone.cmd_trajectory.fly_now = true;
    drone.cmd_trajectory.header.frame_id = drone.local_frame_id;
    drone.cmd_trajectory.dt = dt; // time between trajectory points
    drone.trajectory_pub.publish(drone.cmd_trajectory);
}
