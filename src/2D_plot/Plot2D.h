//
// Created by micha on 2/25/2022.
//

#pragma once

#include "../math/Vec3.h"
#include <memory>
#include <vector>
#include <iostream>
#include <queue>
#include <algorithm>
#include "../tree_structure/Node.h"
#include "../environment_and_objects/Object.h"
#include "../environment_and_objects/World.h"
#include "../path_planning_algorithms/Algorithm.h"
#include "../avoidance/AvoidanceAlgorithm.h"
#include "../motion/Drone.h"
#include <string>

#include"nlohmann/json.hpp"
#include <fstream>

class Plot2D {

public:

    static void write_tree_structure_to_json_file(Node *root,
                                                  const std::string& tree_name,
                                                  const std::string& filename,
                                                  const std::vector<Vec3> &path,
                                                  const Vec3 &goal,
                                                  double goal_radius,
                                                  const std::vector<Object> &obstacles);

    static void write_multiple_trajectories_to_json_file(const std::vector<Drone> &drones,
                                                         const std::string& filename,
                                                         const std::vector<Object> &obstacles,
                                                         const std::string &graph_title);

private:

    static void write_obstacles_to_json_file(nlohmann::json &j_structure,
                                             const std::vector<Object> &obstacles);


};

