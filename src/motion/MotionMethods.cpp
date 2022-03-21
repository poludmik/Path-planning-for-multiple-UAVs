//
// Created by micha on 2/9/2022.
//

#include "MotionMethods.h"

#define PI 3.14159265

void MotionMethods::go_to_point_proportional(Drone &drone, const Vec3& point){

    ros::Rate rate(10);
    mrs_msgs::UavState::ConstPtr cur_uav_state;
    ros::spinOnce();
    rate.sleep();


    while (ros::ok()) {

        if (drone.ready_map[ready_modules::ODOMETRY]) {

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
    // If needed to go along the path for some reason
    //    for (uint32_t i = path.size(); i > 0; --i) {
    //        path[i] = path[i] - path[i - 1];
    //    }
    // std::this_thread::sleep_for(std::chrono::milliseconds(3000));

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
    /*
     * Publish command to fly along the trajectory and wait for the finish.
     * */
    std::vector<mrs_msgs::Reference> array;

    ros::Rate rate(10);

    ros::spinOnce();
    rate.sleep();

    mrs_msgs::UavState::ConstPtr cur_uav_state = drone.uav_state;
    Vec3 start_odom(cur_uav_state->pose.position.x,
                    cur_uav_state->pose.position.y,
                    cur_uav_state->pose.position.z);

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


    std::cout << "Started flying.\n";
    Vec3 curr_pos_odom(cur_uav_state->pose.position.x - start_odom.x,
                         cur_uav_state->pose.position.y - start_odom.y,
                         cur_uav_state->pose.position.z - start_odom.z);

    double heading = Orientation::get_heading_in_rad_from_quaternion(cur_uav_state->pose.orientation.x,
                                                                     cur_uav_state->pose.orientation.y,
                                                                     cur_uav_state->pose.orientation.z,
                                                                     cur_uav_state->pose.orientation.w);
    //if (heading < 0.0) // (-PI, PI> to <0; 2*PI)
    //    heading = heading + PI;

    Vec3 goal_local = path.back();
    goal_local = Orientation::rotate_vector_around_z(goal_local, heading);

    std::cout << "Flying.\n";
    while (Vec3::distance_between_two_vec3(curr_pos_odom, goal_local) > 0.1) {

//        std::cout << "\nOdom_position: ";
//        curr_pos_odom.printout();
//        std::cout << "Goal: ";
//        goal_local.printout();
        cur_uav_state = drone.uav_state;

        curr_pos_odom = Vec3(cur_uav_state->pose.position.x - start_odom.x,
                           cur_uav_state->pose.position.y - start_odom.y,
                           cur_uav_state->pose.position.z - start_odom.z);

        ros::spinOnce();
        rate.sleep();
    }
}
