#include <mrs_msgs/UavState.h>
#include <mrs_msgs/VelocityReferenceStamped.h>

#include <ros/ros.h>

#include <mutex>

mrs_msgs::UavState::ConstPtr uav_state;
std::mutex                   uav_state_mutex;

void odomCallback(mrs_msgs::UavState::ConstPtr const &msg);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "drone_planner");

	size_t uav_id = 1;
	std::string vel_pub_topic  = "/uav" + std::to_string(uav_id) + "/control_manager/velocity_reference";
	std::string odom_sub_topic = "/uav" + std::to_string(uav_id) + "/odometry/uav_state";

	ros::NodeHandle n;
	ros::Subscriber odom_sub  = n.subscribe(odom_sub_topic, 100, odomCallback);
	ros::Publisher  vel_pub   = n.advertise<mrs_msgs::VelocityReferenceStamped>(vel_pub_topic, 100);

	ros::Rate rate(10);
	while(ros::ok())
	{
		// If you need the state, get it like this:
		mrs_msgs::UavState::ConstPtr cur_uav_state;
		{
			std::lock_guard<std::mutex> lock(uav_state_mutex);
			cur_uav_state = uav_state;
		}

		// publish a velocity message - do any velocity calculations here
		mrs_msgs::VelocityReferenceStamped cmd;
		cmd.reference.velocity.x = 2.0*sin(1.0* static_cast<double>(ros::Time::now().toSec())/2/M_PI);
		cmd.reference.velocity.y = 1.0*sin(4.0*static_cast<double>(ros::Time::now().toSec())/2/M_PI);

		std::cout << "publishing: [" << ros::Time::now().toSec() << "] " << cmd.reference.velocity.x << ", " << cmd.reference.velocity.y  << std::endl;
		vel_pub.publish(cmd);

		ros::spinOnce();
		rate.sleep();
	}

	return EXIT_SUCCESS;
}

void odomCallback(mrs_msgs::UavState::ConstPtr const &msg)
{
	std::lock_guard<std::mutex> lock(uav_state_mutex);
	uav_state = msg;
}
