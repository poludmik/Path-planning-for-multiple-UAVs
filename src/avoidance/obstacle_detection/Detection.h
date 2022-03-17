//
// Created by micha on 3/15/2022.
//

#pragma once

#include "../../environment_and_objects/Object.h"
#include "../../motion/Drone.h"

#define PI 3.14159265

class Detection {

public:

    static void get_obstacles_around_the_drone(const Drone &drone, World &drones_world) {

        double radius_obst;
        std::vector<double> sectors = drone.sectors_state->sectors;
        sectors.resize(drone.number_of_sectors);
        double single_angle_sector = 360.0 / drone.number_of_sectors;
        double angle_constant = sin(single_angle_sector * PI / 360.0);
        std::cout << "NumberOfSectors: " << drone.number_of_sectors << "\n";
        double current_angle = 0.0;
        double x;
        double y;

        for (const auto &distance : sectors) {
            if (distance <= 0.0) {
                current_angle += single_angle_sector;
                continue;
            }

            radius_obst = distance * angle_constant;

            // std::cout << current_angle << "\n";
            x = (distance + radius_obst) * cos(current_angle * PI / 180.0); // parameter in radians
            y = (distance + radius_obst) * sin(current_angle * PI / 180.0);

            drones_world.add_obstacle(new Cylinder(radius_obst, Vec3(x, y, -1), 3));
            current_angle += single_angle_sector;
        }
    }


};

