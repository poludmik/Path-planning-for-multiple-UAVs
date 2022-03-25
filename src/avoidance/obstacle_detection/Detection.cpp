//
// Created by micha on 3/15/2022.
//

#include "Detection.h"

void Detection::update_obstacles_around_the_drone(Drone &drone) {

    drone.world->obstacles.clear();

    ros::spinOnce();

    double radius_obst;
    std::vector<double> sectors = drone.sectors_state->sectors;
    sectors.resize(drone.number_of_sectors);
    double single_angle_sector = 360.0 / drone.number_of_sectors;
    double angle_constant = sin(single_angle_sector * PI / 360.0);
    double current_angle = 0.0;
    double x;
    double y;
    double min_obst_radius;

    if (drone.drone_radius <= 1.0)
        min_obst_radius = 1.0 - drone.drone_radius;
    else
        min_obst_radius = 0.5;

    for (auto &distance : sectors) {
        if (distance <= 0.0) {
            current_angle += single_angle_sector;
            continue;
        }
        // 30 sectors and this setup is alright

        radius_obst = std::max(min_obst_radius, std::min(0.8, distance * angle_constant));
        if (radius_obst + drone.drone_radius > distance){
            std::cout << "Cutting the obstacle.**********\n";
            radius_obst = distance - drone.drone_radius - 0.08;
        }

        // std::cout << current_angle << "\n";
        x = (distance) * cos(current_angle * PI / 180.0); // parameter in radians
        y = (distance) * sin(current_angle * PI / 180.0);

        Vec3 obstacle_location(x, y, 0);

        if (Vec3::distance_between_two_vec3(drone.goal_point, obstacle_location) <= (drone.goal_radius + radius_obst) / 1.3) {
            std::cout << ">>>>>Goal inside an obstacle, obstacle removed.<<<<<\n";
            continue;
        }
        obstacle_location.z = -drone.sectors_state->sectors[drone.number_of_sectors];

        drone.world->add_obstacle(new Cylinder(radius_obst, obstacle_location, 3));
        current_angle += single_angle_sector;
    }
}
