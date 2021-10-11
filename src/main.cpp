#include <mrs_msgs/UavState.h>
#include <mrs_msgs/VelocityReferenceStamped.h>
#include <visualization_msgs/Marker.h>
#include <vector>

#include <ros/ros.h>
#include <mutex>

#include "Vec3.h"
#include "World.h"
#include "Object.h"

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
	
//	Vec3 pt1(1.0, 0.0, 0.0);
//	Vec3 pt2;
//	Vec3 pt3 = pt1 + pt2;
	
//	int num = 15;
//	my_world.obstacles.reserve(num);
//
//	for (int i = 0; i < num; ++i) {
//		std::string s = "name" + std::to_string(i);
//		my_world.add_object(s, 0.3, Vec3::random_vec3(-2, 2));
//		my_world.obstacles[i].print_out_info();
//	}
//
//	my_world.add_object("Goal", 1, Vec3(2, 2, 2));
//	auto k = my_world.obstacles.size();
//	my_world.obstacles[k - 1].set_as_a_goal();


//	my_world.add_object("First", 0.6, Vec3(1, 0, 1));
//	my_world.add_object("Second", 1.8, Vec3(1.5, 0, 0));
//	my_world.add_object("Third", 2, Vec3(3, 0, 1));
//	auto k = my_world.obstacles.size();
//	my_world.obstacles[k - 1].set_as_a_goal();
	
//	std::cout << Object::are_intersecting(my_world.obstacles[k-1], my_world.obstacles[1]) << std::endl;
//	std::cout << Object::are_intersecting(my_world.obstacles[1], my_world.obstacles[0]) << std::endl;


	
	my_world.publish_world(vis_pub);
	
	while(ros::ok())
	{
		// get state
		mrs_msgs::UavState::ConstPtr cur_uav_state;
		if (ready)
		{
			std::lock_guard<std::mutex> lock(uav_state_mutex);
			cur_uav_state = uav_state;
//			ROS_INFO("I heard: [%f]", cur_uav_state->pose.position.x);
		}
		
//		std::cout << cur_uav_state->pose.position.x << std::endl;
//		std::cout << "publishing: [" << ros::Time::now().toSec() << "] " << cmd.reference.velocity.x << ", " << cmd.reference.velocity.y  << std::endl;
//		vis_pub.publish(marker0);

		ros::spinOnce();
		rate.sleep();
	}
	return EXIT_SUCCESS;
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
