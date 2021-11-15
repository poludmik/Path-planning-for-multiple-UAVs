#include <mrs_msgs/UavState.h>
#include <mrs_msgs/VelocityReferenceStamped.h>
#include <visualization_msgs/Marker.h>
#include <vector>

#include <ros/ros.h>
#include <mutex>

#include "Vec3.h"
#include "World.h"
#include "Object.h"
#include "Node.h"
#include "RRT_tree.h"
#include "VelocityControllerP.h"


#include "Algorithm.h"
#include "RRTAlgorithm.h"
#include "RRTStarAlgorithm.h"

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
	        ("visualization_marker", 10);

	World my_world;
	
	Vec3 pt_start(-5, 0, 0);
	Vec3 pt_goal(5.0, 0,0);

//    Vec3 rock(4.0, 0, 0.0);
//    my_world.add_obstacle(2, rock);


//    Vec3 rock2(4.0, 0, 2.0);
//    my_world.add_obstacle(2, rock2);
//    Vec3 rock3(4.0, -2, 0.0);
//    my_world.add_obstacle(2, rock3);
//    Vec3 rock4(4.0, -5, 3);
//    my_world.add_obstacle(2, rock4);

    double goal_radius = 1.0;

//	std::vector<Vec3> path = RRT_tree::find_path_to_goal_RRT(&my_world, pt_start, pt_goal, goal_radius, 3);

    RRT_tree tree(pt_start, &my_world, 4);
    std::vector<Vec3> path = tree.find_path(RRTStarAlgorithm(), pt_goal, goal_radius);


    for (const auto& point : path) {
        printf("%lf %lf %lf\n", point.x, point.y, point.z);
        my_world.add_object(0.2, point);
    }

    my_world.add_object(goal_radius, pt_goal);
    my_world.add_object(1.0, pt_start);
    auto k = my_world.objects.size();
    my_world.objects[k - 2].set_as_a_goal();
    my_world.objects[k - 1].set_as_a_start();
    my_world.publish_world(vis_pub);
    World::publish_path(vis_pub, path);





    // to go without flying
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }


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

double clip(double n, double lower, double upper) {
    return std::max(lower, std::min(n, upper));
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
