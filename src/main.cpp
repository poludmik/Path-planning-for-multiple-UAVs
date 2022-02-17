#include <mrs_msgs/UavState.h>
#include <mrs_msgs/VelocityReferenceStamped.h>
#include <mrs_msgs/ReferenceStamped.h>
#include <mrs_msgs/TrajectoryReference.h>
#include <mrs_msgs/Reference.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>

#include <string>

#include <ros/ros.h>
#include <mutex>

#include <fstream>

#include <chrono>
#include <thread>

#include "math/Vec3.h"
#include "environment_and_objects/World.h"
#include "environment_and_objects/Object.h"
#include "tree_structure/Node.h"
#include "tree_structure/RRT_tree.h"
#include "VelocityControllerP.h"

#include "path_planning_algorithms/Algorithm.h"
#include "path_planning_algorithms/RRTAlgorithm.h"
#include "path_planning_algorithms/RRTStarAlgorithm.h"

#include "avoidance/AvoidanceAlgorithm.h"
#include "avoidance/LinearAlgebraIntersetion.h"
#include "avoidance/BinarySearchIntersection.h"

#include "motion/MotionMethods.h"
#include "motion/Drone.h"

bool ready = false;
mrs_msgs::UavState::ConstPtr uav_state;
std::mutex uav_state_mutex;


void odomCallback(const mrs_msgs::UavState::ConstPtr &msg);


int main(int argc, char **argv)
{
	ros::init(argc, argv, "drone_planner");
    // VELOCITY CONTROL
    size_t uav_id = 1;
    std::string vel_pub_topic  = "/uav" + std::to_string(uav_id) + "/control_manager/velocity_reference";
    std::string odom_sub_topic = "/uav" + std::to_string(uav_id) + "/odometry/uav_state";

    ros::NodeHandle n;
    ros::Subscriber odom_sub  = n.subscribe(odom_sub_topic, 100, odomCallback);
    ros::Publisher  vel_pub   = n.advertise<mrs_msgs::VelocityReferenceStamped>(vel_pub_topic, 100);

    ros::Rate rate(10);

    // MARKER RVIZ
    ros::init(argc, argv, "basic_shapes");
    ros::NodeHandle markers_node_publisher;
    ros::Publisher vis_pub = markers_node_publisher.advertise<visualization_msgs::Marker>
            ("visualization_marker", 10);

    ros::NodeHandle markers_array_node_publisher;
    ros::Publisher vis_array_pub = markers_array_node_publisher.advertise<visualization_msgs::MarkerArray>
            ("visualization_marker_array", 100);

	World my_world;

    double goal_radius = 0.5;
    double drone_radius = 0.5;

    Vec3 start1(-3.5, 0, 1);
    Vec3 goal1(3.5, 0, 1);

    Vec3 start2(2.5, 2.5, 1);
    Vec3 goal2(-2.5, -1.5, 1);

    std::vector<Drone> drones;
    Drone dron1(1, start1, goal1, goal_radius, drone_radius);
    drones.push_back(dron1);
    drones.emplace_back(2, start2, goal2, goal_radius, drone_radius);

    Vec3 standing_center(0, 0, 0);
    my_world.add_obstacle("cylinder", 1, standing_center, 5);
    float neighbor_radius = 3;


    for (Drone &drone : drones) {

        RRT_tree tree(drone.start_point, &my_world, neighbor_radius);
        drone.found_trajectory = tree.find_path(RRTStarAlgorithm(), BinarySearchIntersection(), drone.goal_point,
                                                 drone.goal_radius,
                                                 drone.drone_radius);

        for (const auto &point: drone.found_trajectory) {
            printf("%lf %lf %lf\n", point.x, point.y, point.z);
            my_world.add_object(0.2, point);
        }

        my_world.add_object(drone.goal_radius, drone.goal_point);
        my_world.add_object(0.3, drone.start_point);
        auto k = my_world.objects.size();
        my_world.objects[k - 2].set_as_a_goal();
        my_world.objects[k - 1].set_as_a_start();

        my_world.publish_world(vis_array_pub);
        World::publish_path(vis_pub, drone.found_trajectory, std::to_string(drone.uav_id));

        ros::spinOnce();
        rate.sleep();
    }
    return 0;

    while (ros::ok()) {

        if (drones[0].ready and drones[1].ready) {
            // std::lock_guard<std::mutex> lock(drone1.uav_state_mutex);
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }

    //MotionMethods::go_through_a_trajectory(drone1, path, 3);
    //MotionMethods::go_through_a_trajectory(drone2, path, 6);

    std::cout << "The End." << std::endl;
    return EXIT_SUCCESS;
}


void odomCallback(mrs_msgs::UavState::ConstPtr const &msg)
{
	std::lock_guard<std::mutex> lock(uav_state_mutex);
	uav_state = msg;
	ready = true;
}

