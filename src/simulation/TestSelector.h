//
// Created by micha on 3/9/2022.
//
#include <mrs_msgs/UavState.h>
#include <mrs_msgs/VelocityReferenceStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <mutex>
#include <fstream>
#include <chrono>

#include "../math/Vec3.h"
#include "../environment_and_objects/World.h"
#include "../environment_and_objects/Object.h"
#include "../tree_structure/RRT_tree.h"
#include "../path_planning_algorithms/RRTStarAlgorithm.h"
#include "../avoidance/BinarySearchIntersection.h"
#include "../motion/Drone.h"
#include "../motion/Trajectory.h"
#include "../2D_plot/Plot2D.h"
#include "../motion/MotionMethods.h"

#pragma once

enum TestCase {
    FIND_TRAJECTORIES,
    GO_THROUGH_TRAJECTORIES,
    ONE_DRONE_THROUGH_FOREST
};

class TestSelector {

public:
    static void run_simulation(TestCase test_case);

private:
    static void basic_trajectory_search();

    static void fly_through_found_paths();

    static void one_drone_through_forest();
};

