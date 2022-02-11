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

	Vec3 pt_start(-3, 0, 1);
	Vec3 pt_goal(3, 0,1);

    // Vec3 rock(5, 0, -0.3);
    // my_world.add_obstacle(3, rock);

    Vec3 standing_center(0, 0, 0);
    my_world.add_obstacle("cylinder", 2.5, standing_center, 1.4);

//    Vec3 standing_center_above(0.7, 2, 3.5);
//    my_world.add_obstacle("cylinder", 1., standing_center_above, 1.5);
//
//    Vec3 standing1(0, 0, 0);
//    my_world.add_obstacle("cylinder", 1.7, standing1, 5.0);
//
//    Vec3 standing2(-0.7, -2, 0);
//    my_world.add_obstacle("cylinder", 1., standing2, 0.5);
//
//    Vec3 standing5(-0.7, -2, 1);
//    my_world.add_obstacle("cylinder", 1., standing5, 2);
//
//    Vec3 standing6(-0.7, -2, 3.7);
//    my_world.add_obstacle("cylinder", 1., standing6, 1.3);
//
//    Vec3 standing3(1, 4, 0);
//    my_world.add_obstacle("cylinder", 1.5, standing3, 2.0);


    double goal_radius = 0.5;

    //Vec3 drone_center(0, 0, 0);
    //Vec3 obj_center(-3, 1, 0);

    float neighbor_radius = 3;
    float drone_radius = 0.3;
    RRT_tree tree(pt_start, &my_world, neighbor_radius);
    std::vector<Vec3> path = tree.find_path(RRTStarAlgorithm(), LinearAlgebraIntersection(), pt_goal, goal_radius, drone_radius);

    //std::string tree_name = "RRT* with multiple obstacles and neighbor radius:" + std::to_string(neighbor_radius);
    //std::string tree_name = "RRT, binary search with spherical UAV";
    //std::string tree_name = "RRT*: min_N_iters = 2000,  D_max = 1.5,  R_n = " + std::to_string(int(neighbor_radius));
    //RRT_tree::write_tree_structure_to_json_file(tree.root.get(), tree_name,
    //                                          "Created_file.json", path,
    //                                          pt_goal, goal_radius,
    //                                          my_world.obstacles);



    for (const auto& point : path) {
        printf("%lf %lf %lf\n", point.x, point.y, point.z);
        my_world.add_object(0.2, point);
    }

    my_world.add_object(goal_radius, pt_goal);
    my_world.add_object(0.5, pt_start);
    auto k = my_world.objects.size();
    my_world.objects[k - 2].set_as_a_goal();
    my_world.objects[k - 1].set_as_a_start();

    my_world.publish_world(vis_array_pub);
    World::publish_path(vis_pub, path);

    ros::spinOnce();
    rate.sleep();

    return 0;

    Drone drone1(1);
    Drone drone2(2);

    while (ros::ok()) {

        if (drone1.ready and drone2.ready) {
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

