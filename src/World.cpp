//
// Created by misha on 10/10/21.
//

#include "World.h"
#include "Object.h"

void World::add_object(const std::string &name, double radius,
                       const Vec3 &given_coords) {
	objects.emplace_back(this, name, radius, given_coords);
}

void World::add_object(double radius, const Vec3 &given_coords) {
	objects.emplace_back(this, radius, given_coords);
}

void World::print_out_objects() {
	std::cout << std::endl;
	for (auto & obstacle : objects){
		std::cout << obstacle.name << std::endl;
	}
	std::cout << std::endl;
}

void World::publish_world(ros::Publisher &publisher) {

    auto publish_one_array = [&publisher](std::vector<Object> &array) {
        int count = 0;
        for (auto ptr = array.begin(); ptr < array.end(); ptr++){
            visualization_msgs::Marker localMarker;
            fill_out_default_marker(localMarker, count, array[count]);
            ++count;
            while (publisher.getNumSubscribers() < 1) {
                if (!ros::ok()) {
                    std::cout << "Cannot publish, !ros::ok.\n";
                }
                ROS_WARN_ONCE("Waiting for at least one single sub.");
                sleep(1);
            }
            publisher.publish(localMarker);
        }
        std::cout << std::endl;
    };

    publish_one_array(objects);
    publish_one_array(obstacles);
}

World::~World() {
	std::cout << "World instance destroyed." << std::endl;
}

void World::fill_out_default_marker(visualization_msgs::Marker &marker,
				    uint8_t id, const Object &obj) {
	const Vec3 &given_coords = obj.coords;
	double const size = obj.radius;
	const std::string &name = obj.name;
    const bool tmp_start = obj.is_start;
	
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
	marker.scale.x = size * 2;
	marker.scale.y = size * 2;
	marker.scale.z = size * 2;
	marker.color.a = 0.4; // see-through or solid 0 to 1
	if (obj.is_goal) {
		marker.color.r = 0.0;
		marker.color.g = 0.0;
		marker.color.b = 1.0;
	} else if (obj.is_goal) {
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
    } else if (obj.is_obstacle){
        marker.color.r = 0.0;
        marker.color.g = 0.5;
        marker.color.b = 0.5;
	} else {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    }
	marker.lifetime = ros::Duration(20);
}

void World::publish_path(ros::Publisher &publisher, const std::vector<Vec3>& points) {

    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "map"; // "uav1/local_origin";
    line_strip.ns = "path";
    line_strip.header.stamp = ros::Time::now();
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.id = 0;
    line_strip.scale.x = 0.1;
    line_strip.color.g = 1.0;
    line_strip.color.a = 0.5;
    for (const auto & point : points)
    {
        geometry_msgs::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        line_strip.points.push_back(p);
    }
    line_strip.lifetime = ros::Duration(20);
    while (publisher.getNumSubscribers() < 1) {
        if (!ros::ok()) {
            std::cout << "Cannot publish, !ros::ok.\n";
        }
        ROS_WARN_ONCE("Waiting for at least one single sub.");
        sleep(1);
    }
    publisher.publish(line_strip);
}

void World::add_obstacle(double radius, const Vec3 &given_coords) {
    obstacles.emplace_back(this, radius, given_coords);
}

void World::add_obstacle(const std::string &name, double radius, const Vec3 &given_coords) {
    obstacles.emplace_back(this, name, radius, given_coords);
}

