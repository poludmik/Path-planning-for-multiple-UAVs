#include <mrs_msgs/UavState.h>
#include <mrs_msgs/VelocityReferenceStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>

#include <string>

#include <ros/ros.h>
#include <mutex>

#include<nlohmann/json.hpp>
#include <fstream>

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

bool ready = false;
mrs_msgs::UavState::ConstPtr uav_state;
std::mutex uav_state_mutex;


void odomCallback(const mrs_msgs::UavState::ConstPtr &msg);

void go_to_point(const Vec3& point,
                 mrs_msgs::VelocityReferenceStamped cmd,
                 const ros::Publisher& vel_pub);

double clip(double n, double lower, double upper);

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
	mrs_msgs::VelocityReferenceStamped cmd;
	std::vector<bool> direction_state{true, false, false, false}; // right, forward, left, forward
	
	cmd.reference.velocity.x = 0.5;
	cmd.reference.velocity.y = 0;
	
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
   // my_world.add_obstacle(2, rock7);
    Vec3 rock8(5.3, -6, 0.3);
    my_world.add_obstacle(1.1, rock8);
    Vec3 rock9(4.9, 6, -1);
    my_world.add_obstacle(1.1, rock9);
    Vec3 rock10(0, -5.6, 2);
    my_world.add_obstacle(2.3, rock10);
    Vec3 rock11(-0.1, -2, 0.5);
    my_world.add_obstacle(0.7, rock11);
    Vec3 rock12(9.8, 4.8, 0);
    my_world.add_obstacle(1.2, rock12);
    Vec3 rock13(-1, 4.7, 2);
    my_world.add_obstacle(1.5, rock13);
    Vec3 rock14(-2, 2.4, -2);
    my_world.add_obstacle(0.95, rock14);

    /*
    Vec3 rock(5, 0.2, 0.0);
    //my_world.add_obstacle(1, rock);
    Vec3 rock2(5.0, 3, 0.0);
    my_world.add_obstacle(1, rock2);
    Vec3 rock3(5, -3, 0.0);
    my_world.add_obstacle(1, rock3);
    Vec3 rock4(3, -1, 0);
    //my_world.add_obstacle(1, rock4);
    Vec3 rock5(3, 1.5, 0);
    my_world.add_obstacle(1, rock5);
    Vec3 rock6(8, 1, 0);
    my_world.add_obstacle(1, rock6);
    Vec3 rock7(8, -2, 0);
    my_world.add_obstacle(2, rock7);
    Vec3 rock8(5.3, -6, 0);
    my_world.add_obstacle(1.1, rock8);
    Vec3 rock9(4.9, 6, 0);
    my_world.add_obstacle(1.1, rock9);
    Vec3 rock10(0, -5.6, 0);
    my_world.add_obstacle(2.3, rock10);
    Vec3 rock11(-0.1, -2, 0);
    my_world.add_obstacle(0.7, rock11);
    Vec3 rock12(9.8, 4.8, 0);
    my_world.add_obstacle(1.2, rock12);
    Vec3 rock13(-1, 4.7, 0);
    my_world.add_obstacle(1.5, rock13);
    Vec3 rock14(-2, 2.4, 0);
    my_world.add_obstacle(0.95, rock14);

    Vec3 nrock(5, 0.2, 0.0);
    my_world.add_obstacle(0.5, nrock);
    Vec3 nrock2(5.0, 3, 0.0);
    my_world.add_obstacle(0.5, nrock2);
    Vec3 nrock3(5, -3, 0.0);
    my_world.add_obstacle(0.5, nrock3);
    Vec3 nrock4(3, -1, 0);
    my_world.add_obstacle(0.5, nrock4);
    Vec3 nrock5(3, 1.5, 0);
    my_world.add_obstacle(0.5, nrock5);
    Vec3 nrock6(8, 1, 0);
    my_world.add_obstacle(0.5, nrock6);
    Vec3 nrock7(8, -2, 0);
    my_world.add_obstacle(1.5, nrock7);
    Vec3 nrock8(5.3, -6, 0);
    my_world.add_obstacle(0.6, nrock8);
    Vec3 nrock9(4.9, 6, 0);
    my_world.add_obstacle(0.6, nrock9);
    Vec3 nrock10(0, -5.6, 0);
    my_world.add_obstacle(1.8, nrock10);
    Vec3 nrock11(-0.1, -2, 0);
    my_world.add_obstacle(0.2, nrock11);
    Vec3 nrock12(9.8, 4.8, 0);
    my_world.add_obstacle(0.7, nrock12);
    Vec3 nrock13(-1, 4.7, 0);
    my_world.add_obstacle(1, nrock13);
    Vec3 nrock14(-2, 2.4, 0);
    my_world.add_obstacle(0.45, nrock14);
*/

/*
    Vec3 rock(6, 0.2, 0.0);
    my_world.add_obstacle(1.5, rock);
    Vec3 rock2(3, 2.2, 0.0);
    my_world.add_obstacle(1.5, rock2);
    Vec3 rock3(8, -3.2, 0.0);
    my_world.add_obstacle(1, rock3);
    Vec3 rock4(7.7, 3.5, 0.0);
    my_world.add_obstacle(2, rock4);
    Vec3 rock5(2, -3, 0.0);
    my_world.add_obstacle(2, rock5);
    Vec3 rock6(5.2, -5.7, 0.0);
    my_world.add_obstacle(2, rock6);
    Vec3 rock7(-0.3, 6.7, 0.0);
    my_world.add_obstacle(2.5, rock7);
    Vec3 rock8(12, -7.6, 0.0);
    my_world.add_obstacle(1.7, rock8);
    Vec3 rock9(13, 1, 0.0);
    my_world.add_obstacle(1.3, rock9);
    Vec3 rock10(-2.5, -7.5, 0.0);
    my_world.add_obstacle(1.6, rock10);

    Vec3 nrock(6, 0.2, 0.0);
    my_world.add_obstacle(1.0, nrock);
    Vec3 nrock2(3, 2.2, 0.0);
    my_world.add_obstacle(1.0, nrock2);
    Vec3 nrock3(8, -3.2, 0.0);
    my_world.add_obstacle(0.5, nrock3);
    Vec3 nrock4(7.7, 3.5, 0.0);
    my_world.add_obstacle(1.5, nrock4);
    Vec3 nrock5(2, -3, 0.0);
    my_world.add_obstacle(1.5, nrock5);
    Vec3 nrock6(5.2, -5.7, 0.0);
    my_world.add_obstacle(1.5, nrock6);
    Vec3 nrock7(-0.3, 6.7, 0.0);
    my_world.add_obstacle(2.0, nrock7);
    Vec3 nrock8(12, -7.6, 0.0);
    my_world.add_obstacle(1.2, nrock8);
    Vec3 nrock9(13, 1, 0.0);
    my_world.add_obstacle(0.8, nrock9);
    Vec3 nrock10(-2.5, -7.5, 0.0);
    my_world.add_obstacle(1.1, nrock10);
   */

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

    return EXIT_SUCCESS;







    Vec3 start_position(0,0,0);

    while (ros::ok()) {
        mrs_msgs::UavState::ConstPtr cur_uav_state;
        if (ready) {
            std::lock_guard<std::mutex> lock(uav_state_mutex);
            cur_uav_state = uav_state;
            start_position = Vec3(cur_uav_state->pose.position.x, cur_uav_state->pose.position.y,
                                                           cur_uav_state->pose.position.z);
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }

    for (auto& point : path) {
        point = point + start_position;
    }

    bool first_iter = true;

    for (const auto& point : path) {

//        std::cout << "next_goal = " << point.x << " " << point.y << "\n";

        while (ros::ok()) {
            mrs_msgs::UavState::ConstPtr cur_uav_state;
            if (ready) {

                std::lock_guard<std::mutex> lock(uav_state_mutex);
                cur_uav_state = uav_state;

                if (first_iter) {
                    first_iter = false;
                }

                Vec3 curr_position = Vec3(cur_uav_state->pose.position.x, cur_uav_state->pose.position.y,
                                         cur_uav_state->pose.position.z);

                Vec3 error = point - curr_position;

                if (error.norm() > 0.2) {
//                    std::cout << uav_state->pose.position.x << "\n";
                    cmd.reference.velocity.x = error.x * 0.2;
                    cmd.reference.velocity.y = error.y * 0.2;
                    cmd.reference.velocity.z = error.z * 0.2;

                    vel_pub.publish(cmd);
                } else {
                    break;
                }
            }

//		std::cout << uav_state->pose.position.x << std::endl;
//		std::cout << "publishing: [" << ros::Time::now().toSec() << "] " << cmd.reference.velocity.x << ", " << cmd.reference.velocity.y  << std::endl;
//		vis_pub.publish(marker0);

            ros::spinOnce();
            rate.sleep();
        }
    }

    return EXIT_SUCCESS;
}

void go_to_point(const Vec3& point,
                 mrs_msgs::VelocityReferenceStamped cmd,
                 const ros::Publisher& vel_pub){

    Vec3 e = point - Vec3(uav_state->pose.position.x, uav_state->pose.position.y, uav_state->pose.position.z);
    ros::Rate rate(10);
    while (e.norm() > 0.2) {

        std::cout << uav_state->pose.position.x << "\n";
        cmd.reference.velocity.x = e.x * 0.1;
        cmd.reference.velocity.y = e.y * 0.1;
        cmd.reference.velocity.z = e.z * 0.1;

        vel_pub.publish(cmd);

        e = point - Vec3(uav_state->pose.position.x, uav_state->pose.position.y, uav_state->pose.position.z);

        rate.sleep();
    }
}

//mrs_msgs::VelocityReferenceStamped &cmd,
//cmd.reference.velocity.y = 0;
//vel_pub.publish(cmd);

void odomCallback(mrs_msgs::UavState::ConstPtr const &msg)
{
	std::lock_guard<std::mutex> lock(uav_state_mutex);
	uav_state = msg;
	ready = true;
}
