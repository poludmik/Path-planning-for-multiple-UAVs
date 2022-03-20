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
    // ::cout << "NumberOfSectors: " << drone.number_of_sectors << "\n";
    double current_angle = 0.0;
    double x;
    double y;

    for (const auto &distance : sectors) {
        if (distance <= 0.0) {
            current_angle += single_angle_sector;
            continue;
        }

        radius_obst = std::max(0.4, std::min(0.8, distance * angle_constant));

        // std::cout << current_angle << "\n";
        x = (distance) * cos(current_angle * PI / 180.0); // parameter in radians
        y = (distance) * sin(current_angle * PI / 180.0);

        drone.world->add_obstacle(new Cylinder(radius_obst, Vec3(x, y, -drone.sectors_state->sectors[drone.number_of_sectors]), 3));
        current_angle += single_angle_sector;
    }
}
