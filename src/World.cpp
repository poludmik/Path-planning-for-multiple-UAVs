//
// Created by misha on 10/10/21.
//

#include "World.h"
#include "Object.h"

void World::add_object(const std::string &name, double radius,
                       const Vec3 &given_coords) {
	obstacles.emplace_back(this, name, radius, given_coords);
}

void World::add_object(double radius, const Vec3 &given_coords) {
	obstacles.emplace_back(this, radius, given_coords);
}

void World::print_out_objects() {
	std::cout << std::endl;
	for (auto & obstacle : obstacles){
		std::cout << obstacle.name << std::endl;
	}
	std::cout << std::endl;
}

void World::publish_world(ros::Publisher &publisher) {
	int count = 0;
	
	std::cout << std::endl;
	for (auto ptr = obstacles.begin(); ptr < obstacles.end(); ptr++){
		visualization_msgs::Marker localMarker;
		fill_out_default_marker(localMarker, count, obstacles[count]);
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

World::~World() {
	std::cout << "World instance destroyed." << std::endl;
}

void World::fill_out_default_marker(visualization_msgs::Marker &marker,
				    uint8_t id, const Object &obj) {
	const Vec3 &given_coords = obj.coords;
	double const size = obj.radius;
	const std::string &name = obj.name;
	
	marker.header.frame_id = "map"; // "uav1/local_origin";
	marker.header.stamp = ros::Time::now();
	marker.ns = name;
	marker.id = id;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = given_coords.x;
	marker.pose.position.y = given_coords.y;
	marker.pose.position.z = given_coords.z;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = size;
	marker.scale.y = size;
	marker.scale.z = size;
	marker.color.a = 0.4; // see-through or solid 0 to 1
	if (obj.is_goal) {
		marker.color.r = 0.0;
		marker.color.g = 0.0;
		marker.color.b = 1.0;
	} else {
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
	}
	marker.lifetime = ros::Duration(20);
}
