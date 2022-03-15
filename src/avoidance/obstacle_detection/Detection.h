//
// Created by micha on 3/15/2022.
//

#pragma once

#include "../../environment_and_objects/Object.h"
#include "../../motion/Drone.h"

#define PI 3.14159265

class Detection {

public:

    static std::vector<std::shared_ptr<Object>> get_obstacles_around_the_drone(const Drone &drone) {

        std::vector<std::shared_ptr<Object>> found_obstacles;
        double radius_obst = 0.3;

        std::vector<double> sectors = drone.sectors_state->sectors;
        sectors.resize(drone.number_of_sectors);
        double single_angle_sector = 360.0 / drone.number_of_sectors;
        std::cout << "NumberOfSectors: " << drone.number_of_sectors << "\n";
        double current_angle = 0.0;
        double x;
        double y;

        for (const auto &distance : sectors) {
            if (distance <= 0.0) {
                current_angle += single_angle_sector;
                continue;
            }
            std::cout << current_angle << "\n";
            x = distance * cos(current_angle * PI / 180.0); // parameter in radians
            y = distance * sin(current_angle * PI / 180.0);

            found_obstacles.emplace_back(new Cylinder(radius_obst, Vec3(x, y, -1), 3));
            current_angle += single_angle_sector;
        }

        return found_obstacles;
    }

};

