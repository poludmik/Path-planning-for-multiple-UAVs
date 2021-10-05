#include <mrs_msgs/UavState.h>
#include <mrs_msgs/VelocityReferenceStamped.h>
#include <visualization_msgs/Marker.h>
#include <vector>

#include <thread>
#include <ros/ros.h>

#include <condition_variable>
#include <mutex>

std::condition_variable cv;
bool ready = false;
mrs_msgs::UavState::ConstPtr uav_state;
std::mutex uav_state_mutex;


void odomCallback(const mrs_msgs::UavState::ConstPtr &msg);

void snakeMoves(mrs_msgs::VelocityReferenceStamped &cmd, std::vector<bool> &direction_state);

void setDefaultLineMarker(visualization_msgs::Marker& marker,
		      float x, float y,
		      float z, float size);

class World;
class Object;

class Object{
public:
    
    std::string name;
    double radius;
    double x;
    double y;
    double z;
    
    void print_out_info() const{
	    std::cout << "name: " << name << std::endl;
	    std::cout << "x: " << x << std::endl;
	    std::cout << "y: " << y << std::endl;
	    std::cout << "z: " << z << std::endl;
    }
    
    Object(World* pBigWorld, const std::string &name, double radius,
           double x,
           double y,
           double z) {
	    this->pBigWorld = pBigWorld;
	    this->name = name;
	    this->radius = radius;
	    this->x = x;
	    this->y = y;
	    this->z = z;
	    std::cout << "Object '" << name << "' has been created." << std::endl;
    }
    
    Object(World* pBigWorld, double radius, double x, double y, double z) {
	    this->pBigWorld = pBigWorld;
	    this->name = "obj";
	    this->radius = radius;
	    this->x = x;
	    this->y = y;
	    this->z = z;
	    std::cout << "Object '" << "obj" << "' has been created." << std::endl;
    }
    
    Object (const Object& obj) { // copy constructor
	    pBigWorld = obj.pBigWorld;
	    name = obj.name;
	    radius = obj.radius;
	    x = obj.x;
	    y = obj.y;
	    z = obj.z;
    }
    
    ~Object() {
	    std::cout << "Object '" << name << "' has been destroyed." << std::endl;
    }

private:
    World* pBigWorld;
};


class World{
public:
    
    // Data Members
    std::vector<Object> obstacles;
    
    void add_object(const std::string &name, double radius, double x, double y, double z){
	    obstacles.emplace_back(this, name, radius, x, y, z);
    }
    
    void add_object(double radius, double x, double y, double z){
	    obstacles.emplace_back(this, radius, x, y, z);
    }
    
    void print_out_objects(){
	    std::cout << std::endl;
	    for (auto & obstacle : obstacles){
		    std::cout << obstacle.name << std::endl;
	    }
	    std::cout << std::endl;
    }
    
    void publish_world(ros::Publisher &publisher){
	    int count = 0;
	    float size = 0.5;

	    std::cout << std::endl;
	    for (auto ptr = obstacles.begin(); ptr < obstacles.end(); ptr++){
		    visualization_msgs::Marker localMarker;
		    fill_out_default_marker(localMarker, count, ptr->x, ptr->y, ptr->z, size, ptr->name);
		    ++count;
//		    std::cout << "publishing " << ptr->name << " "<< count << " marker\n";
		    while (publisher.getNumSubscribers() < 1)
		    {
			    if (!ros::ok())
			    {
				    std::cout << "Cannot publish, !ros::ok.\n";
			    }
			    ROS_WARN_ONCE("Waiting for at least one single sub.");
			    sleep(1);
		    }
		    publisher.publish(localMarker);
	    }
	    std::cout << std::endl;
    }
    
    static void fill_out_default_marker(visualization_msgs::Marker& marker,
			                        uint8_t const id,
                                                double const x,
                                                double const y,
                                                double const z,
                                                double const size,
                                                const std::string &name){
	    marker.header.frame_id = "map"; // "uav1/local_origin";
	    marker.header.stamp = ros::Time::now();
	    marker.ns = name;
	    marker.id = id;
	    marker.type = visualization_msgs::Marker::SPHERE;
	    marker.action = visualization_msgs::Marker::ADD;
	    marker.pose.position.x = x;
	    marker.pose.position.y = y;
	    marker.pose.position.z = z;
	    marker.pose.orientation.x = 0.0;
	    marker.pose.orientation.y = 0.0;
	    marker.pose.orientation.z = 0.0;
	    marker.pose.orientation.w = 1.0;
	    marker.scale.x = size;
	    marker.scale.y = size;
	    marker.scale.z = size;
	    marker.color.a = 0.4; // see-through or solid 0 to 1
	    marker.color.r = 1.0;
	    marker.color.g = 0.0;
	    marker.color.b = 0.0;
	    marker.lifetime = ros::Duration(100);
	    }
    
    World() { // constructor
	    // object initialization
    }
    
    ~World() {
	    std::cout << "World instance destroyed." << std::endl;
    };
};


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
	int num = 3;
	my_world.obstacles.reserve(num);

	for (int i = 0; i < num; ++i) {
		std::string s = "name" + std::to_string(i);
		my_world.add_object(s, (double) i, (double) i, ((double) i )/ 2, 0.0);
	}

	my_world.publish_world(vis_pub);
	
	while(ros::ok())
	{
		
//		my_world.publish_world(vis_pub);
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

void odomCallback(mrs_msgs::UavState::ConstPtr const &msg)
{
	std::lock_guard<std::mutex> lock(uav_state_mutex);
	uav_state = msg;
	ready = true;
}

void setDefaultLineMarker(visualization_msgs::Marker& marker,
                         float const x,
                         float const y,
                         float const z,
                         float const size){
	marker.header.frame_id = "map"; // "uav1/local_origin";
	marker.header.stamp = ros::Time::now();
	marker.ns = "basic_shapes";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 1;
	marker.pose.position.y = 0;
	marker.pose.position.z = 1.2;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = size;
	marker.scale.y = size;
	marker.scale.z = size;
	marker.color.a = 0.3; // see-through or solid 1
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	
	marker.color.r = 1.0f;
	marker.color.a = 1.0;
	// Create the vertices for the points and lines
	for (uint32_t i = 0; i < 10; ++i) {
		float f = 0;
		float g = 1;
		
		geometry_msgs::Point p;
		p.x = (int32_t) i;
		p.y = f;
		p.z = g;
		
		marker.points.push_back(p);
	}
//	marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
}

void snakeMoves(mrs_msgs::VelocityReferenceStamped &cmd, std::vector<bool> &direction_state){
	uint8_t counter = 41;
	if (counter > 40) {
		if (direction_state.at(0)) {
			cmd.reference.velocity.x = 0.6;
			cmd.reference.velocity.y = 0;
		}
		else if (direction_state.at(1) || direction_state.at(3)) {
			cmd.reference.velocity.x = 0;
			cmd.reference.velocity.y = 0.5;
		}
		else if (direction_state.at(2)) {
			cmd.reference.velocity.x = -0.6;
			cmd.reference.velocity.y = 0;
		}
		counter = 0;
		short int i = 0;
		std::rotate(direction_state.begin(), direction_state.begin()+1, direction_state.end());
//			std::cout << "x = " << cmd.reference.velocity.x << ", y = " << cmd.reference.velocity.y << std::endl;
	}
	++counter;
//	vel_pub.publish(cmd);
};


