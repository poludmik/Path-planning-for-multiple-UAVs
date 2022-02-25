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

#include "math/Vec3.h"
#include "environment_and_objects/World.h"
#include "environment_and_objects/Object.h"
#include "tree_structure/RRT_tree.h"
#include "path_planning_algorithms/RRTStarAlgorithm.h"
#include "avoidance/BinarySearchIntersection.h"
#include "motion/Drone.h"
#include "motion/Trajectory.h"
#include "2D_plot/Plot2D.h"

mrs_msgs::UavState::ConstPtr uav_state;
std::mutex uav_state_mutex;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "drone_planner");
    // VELOCITY CONTROL
    size_t uav_id = 1;
    std::string vel_pub_topic  = "/uav" + std::to_string(uav_id) + "/control_manager/velocity_reference";

    ros::NodeHandle n;
    ros::Publisher  vel_pub   = n.advertise<mrs_msgs::VelocityReferenceStamped>(vel_pub_topic, 100);

    ros::Rate rate(10);

    // MARKER RVIZ
    ros::init(argc, argv, "basic_shapes");
    ros::NodeHandle markers_node_publisher;
    ros::Publisher vis_pub = markers_node_publisher.advertise<visualization_msgs::Marker>
            ("visualization_marker", 10);

    ros::NodeHandle markers_array_node_publisher;
    ros::Publisher vis_array_pub = markers_array_node_publisher.advertise<visualization_msgs::MarkerArray>
            ("visualization_marker_array", 300);

	World my_world;

    double goal_radius = 0.5;
    double drone_radius = 0.5;

    Vec3 start1(-3.5, 2, 1);
    Vec3 goal1(3.5, 2, 1);

    Vec3 start2(-3.4, 0.8, 1);
    Vec3 goal2(3.5, -0.8, 1);

    Vec3 start3(-3.5, -0.8, 1);
    Vec3 goal3(3.5, 0.8,1);

    Vec3 start4(-3.5, -2.3, 1);
    Vec3 goal4(3.5, -2.3, 1);

    std::vector<Drone> drones;
    Drone dron1(1, start1, goal1, goal_radius, drone_radius);
    drones.push_back(dron1);
    drones.emplace_back(2, start2, goal2, goal_radius, drone_radius);
    drones.emplace_back(3, start3, goal3, goal_radius, drone_radius);
    drones.emplace_back(4, start4, goal4, goal_radius, drone_radius);

    Vec3 standing_center(0, 0, 0);
    my_world.add_obstacle("cylinder", 0.4, standing_center, 3.5);
    Vec3 standing_center1(0, 2, 0);
    my_world.add_obstacle("cylinder", 0.3, standing_center1, 4);
    Vec3 standing_center2(0, -2, 0);
    my_world.add_obstacle("cylinder", 0.35, standing_center2, 3);
    Vec3 standing_center3(-2, 1, 0);
    //my_world.add_obstacle("cylinder", 0.4, standing_center3, 5);
    Vec3 standing_center4(-2, -0, 0);
    my_world.add_obstacle("cylinder", 0.4, standing_center4, 2);
    Vec3 standing_center5(2, 1, 0);
    my_world.add_obstacle("cylinder", 0.4, standing_center5, 5);


    float neighbor_radius = 3;

    for (Drone &drone : drones) {

        RRT_tree tree(drone.start_point, &my_world, neighbor_radius);
        drone.found_path = tree.find_path(RRTStarAlgorithm(), BinarySearchIntersection(), drone.goal_point,
                                          drone.goal_radius,
                                          drone.drone_radius);

        drone.trajectory = Trajectory(drone.found_path, 0.2, 0.3);
    }

    // performance mode is better for 3D
    Trajectory::resolve_all_conflicts_with_new_trajectories(my_world, drones, false);

    for (Drone &drone : drones) {

        for (const auto &point_in_time: drone.trajectory.time_points) {
            // printf("%lf\n", point_in_time.second);
            //printf("%lf %lf %lf\n", point_in_time.first.x, point_in_time.first.y, point_in_time.first.z);
            my_world.add_object(0.05, point_in_time.first);
        }

        my_world.add_object(drone.goal_radius, drone.goal_point);
        my_world.add_object(drone.drone_radius, drone.start_point);
        auto k = my_world.objects.size();
        my_world.objects[k - 2].set_as_a_goal();
        my_world.objects[k - 1].set_as_a_start();

        World::publish_trajectory(vis_pub, drone.trajectory, std::to_string(drone.uav_id));

        ros::spinOnce();
        rate.sleep();
    }

    std::string file_name = "Multiple_trajectories.json";
    std::string plot_title = "Multiple paths";
    Plot2D::write_multiple_trajectories_to_json_file(drones, file_name, my_world.obstacles, plot_title);

     my_world.publish_world(vis_array_pub);

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


