//
// Created by micha on 2/9/2022.
//

#include "Drone.h"

Drone::Drone(const size_t uav_id, const Vec3 &start_point, const Vec3 &goal_point, const double goal_radius, const double drone_radius) {
    this->uav_id = uav_id;
    this->start_point = start_point;
    this->goal_point = goal_point;
    this->goal_radius = goal_radius;
    this->drone_radius = drone_radius;
    this->number_of_sectors = 0;
    this->world = std::unique_ptr<World>(new World("uav" + std::to_string(uav_id) + "/fcu"));

    std::string odom_sub_topic = "/uav" + std::to_string(uav_id) + "/odometry/uav_state";
    std::string bumper_sub_topic = "/uav" + std::to_string(uav_id) +  "/bumper/obstacle_sectors";
    odom_sub  = n.subscribe(odom_sub_topic, 100, &Drone::odomCallback, this);
    bumper_sub  = n.subscribe(bumper_sub_topic, 1, &Drone::bumperCallback, this);

    std::string vel_pub_topic  = "/uav" + std::to_string(uav_id) + "/control_manager/velocity_reference";
    std::string reference_pub_topic  = "/uav" + std::to_string(uav_id) + "/control_manager/reference";
    std::string trajectory_pub_topic  = "/uav" + std::to_string(uav_id) + "/control_manager/trajectory_reference";
    vel_pub   = n.advertise<mrs_msgs::VelocityReferenceStamped>(vel_pub_topic, 100);
    goto_pub = n.advertise<mrs_msgs::ReferenceStamped>(reference_pub_topic, 100);
    trajectory_pub = n.advertise<mrs_msgs::TrajectoryReference>(trajectory_pub_topic, 100);

    local_frame_id = "uav" + std::to_string(uav_id) + "/fcu";
    global_frame_id = "uav" + std::to_string(uav_id) + "/local_origin";
}

void Drone::odomCallback(const mrs_msgs::UavState_<std::allocator<void>>::ConstPtr &msg) {
    std::mutex uav_state_mutex;
    std::lock_guard<std::mutex> lock(uav_state_mutex);
    uav_state = msg;
    ready_map[ODOMETRY] = true;
}

void Drone::bumperCallback(const mrs_msgs::ObstacleSectors_<std::allocator<void>>::ConstPtr &msg) {
    std::mutex uav_bumper_mutex;
    std::lock_guard<std::mutex> lock(uav_bumper_mutex);
    sectors_state = msg;
    ready_map[BUMPER] = true;
    if (!number_of_sectors) number_of_sectors = sectors_state->n_horizontal_sectors;
}

bool Drone::isReady() const { // All the ready_map values are true.
    return (std::all_of(std::begin(ready_map),std::end(ready_map),[](const auto &pair) {
                return pair.second;
            }
    ));
}

