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
    std::string reference_pub_topic  = "/uav" + std::to_string(uav_id) + "/control_manager/reference";
    std::string trajectory_pub_topic  = "/uav" + std::to_string(uav_id) + "/control_manager/trajectory_reference";
	std::string odom_sub_topic = "/uav" + std::to_string(uav_id) + "/odometry/uav_state";

	ros::NodeHandle n;
	ros::Subscriber odom_sub  = n.subscribe(odom_sub_topic, 100, odomCallback);
	ros::Publisher  vel_pub   = n.advertise<mrs_msgs::VelocityReferenceStamped>(vel_pub_topic, 100);
    ros::Publisher  goto_pub = n.advertise<mrs_msgs::ReferenceStamped>(reference_pub_topic, 100);
    ros::Publisher  trajectory_pub = n.advertise<mrs_msgs::TrajectoryReference>(trajectory_pub_topic, 100);

	ros::Rate rate(10);
	mrs_msgs::VelocityReferenceStamped cmd;
    mrs_msgs::ReferenceStamped cmd_goto;
    mrs_msgs::TrajectoryReference cmd_traj;


    // MARKER RVIZ
	ros::init(argc, argv, "basic_shapes");
	ros::NodeHandle markers_node_publisher;
	ros::Publisher vis_pub = markers_node_publisher.advertise<visualization_msgs::Marker>
	        ("visualization_marker", 100);

    ros::NodeHandle markers_array_node_publisher;
    ros::Publisher vis_array_pub = markers_array_node_publisher.advertise<visualization_msgs::MarkerArray>
            ("visualization_marker_array", 100);

	World my_world;
	
	Vec3 pt_start(0, 0, 0);
	Vec3 pt_goal(10, 0,0);

    Vec3 rock(5, -0.5, -0.3);
    my_world.add_obstacle(3, rock);
    Vec3 rock2(5.0, 3, 0.0);
    my_world.add_obstacle(1, rock2);
    Vec3 rock3(5, -3, 1.0);
    my_world.add_obstacle(1, rock3);
    Vec3 rock4(3, -1, 0);
    my_world.add_obstacle(1, rock4);
    Vec3 rock5(3, 1.5, -1);
    my_world.add_obstacle(1, rock5);
    Vec3 rock6(8, 1, -3);
    my_world.add_obstacle(1, rock6);
    Vec3 rock7(8, -2, 5);

    double goal_radius = 0.8;

    Vec3 drone_center(0, 0, 0);
    Vec3 obj_center(-3, 1, 0);
//    std::cout << AvoidanceAlgorithm::DoesIntersectByBinaryAvoidance(pt_start, pt_goal, 0.5, obj_center, 0.6) << "\n";

//	std::vector<Vec3> path = RRT_tree::find_path_to_goal_RRT(&my_world, pt_start, pt_goal, goal_radius, 3);

    float neighbor_radius = 3;
    float drone_radius = 0.6;
    RRT_tree tree(pt_start, &my_world, neighbor_radius);
    std::vector<Vec3> path = tree.find_path(RRTStarAlgorithm(), BinarySearchIntersection(), pt_goal, goal_radius, drone_radius);

//    std::string tree_name = "RRT* with multiple obstacles and neighbor radius:" + std::to_string(neighbor_radius);
    std::string tree_name = "RRT, binary search with spherical UAV";
    //std::string tree_name = "RRT*: min_N_iters = 2000,  D_max = 1.5,  R_n = " + std::to_string(int(neighbor_radius));
    RRT_tree::write_tree_structure_to_json_file(tree.root.get(), tree_name,
                                                "Created_file.json", path,
                                                pt_goal, goal_radius,
                                                my_world.obstacles);


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


    mrs_msgs::Reference reference;
    reference.position.x = -5;
    reference.position.y = -5;
    reference.position.z = -7;

    std::vector<mrs_msgs::Reference> array;
    array.push_back(reference);

    cmd_traj.points = array;
    cmd_traj.fly_now = true;
    cmd_traj.header.frame_id = "uav1/fcu";
    // trajectory_pub.publish(cmd_traj);


    cmd_goto.reference.position.x = -5.0;
    cmd_goto.reference.position.y = -5.0;
    cmd_goto.reference.position.z = 2.0;
    cmd_goto.header.frame_id = "uav1/fcu";
    cmd_goto.reference.heading = 0.0;
    cmd_goto.header.stamp = ros::Time::now();
    //goto_pub.publish(cmd_goto);


    Drone drone(1);

//    MotionMethods::go_to_the_point(drone, Vec3(-10, -10, -8.0));

    Vec3 start_position;
    mrs_msgs::UavState::ConstPtr cur_uav_state;

    while (ros::ok()) {

        if (drone.ready) {
            std::lock_guard<std::mutex> lock(drone.uav_state_mutex);
            cur_uav_state = drone.uav_state;
            start_position = Vec3(cur_uav_state->pose.position.x,
                                  cur_uav_state->pose.position.y,
                                  cur_uav_state->pose.position.z);
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }

    std::cout << "start_position: " << start_position.x << " " << start_position.y << " " << start_position.z << "\n";

    MotionMethods::go_to_the_point(drone, Vec3(5, 0, 2.0));

    for (auto &point : path){
        point = point + start_position;
    }

    for (const auto& point : path) {
        std::cout << "point: " << point.x << " " << point.y << " " << point.z << "\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        MotionMethods::go_to_the_point(drone, point);
    }

    std::cout << "The End." << std::endl;
    return EXIT_SUCCESS;
}


void odomCallback(mrs_msgs::UavState::ConstPtr const &msg)
{
	std::lock_guard<std::mutex> lock(uav_state_mutex);
	uav_state = msg;
	ready = true;
}
