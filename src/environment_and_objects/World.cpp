//
// Created by misha on 10/10/21.
//

#include "World.h"
#include <visualization_msgs/MarkerArray.h>
#include "../motion/Trajectory.h"

void World::publish_world(const ros::Publisher &publisher) const {
    auto publish_one_array = [&](const std::vector<std::shared_ptr<Object>> &array) {
        int count = 0;
        visualization_msgs::MarkerArray markerArray;
        ros::Rate rate(100);
        for (auto ptr = array.begin(); ptr < array.end(); ptr++){
            visualization_msgs::Marker localMarker;
            //array[count].print_out_info();
            fill_out_default_marker(localMarker, count, *array[count]);
            ++count;
            while (publisher.getNumSubscribers() < 1) {
                if (!ros::ok()) {
                    std::cout << "Cannot publish, !ros::ok.\n";
                }
                ROS_WARN_ONCE("Waiting for at least one single sub.");
                sleep(1);
            }
            markerArray.markers.push_back(localMarker);
            //publisher.publish(localMarker);
        }
        std::cout << std::endl;
        publisher.publish(markerArray);
        ros::spinOnce();
        rate.sleep();
    };

    //publish_one_array(objects);
    //publish_one_array(obstacles);
    std::vector<std::shared_ptr<Object>> both = obstacles;
    both.insert(both.end(), objects.begin(), objects.end());

    publish_one_array(both);
    std::cout << "published\n";
}

World::~World() {
	std::cout << "World instance destroyed." << std::endl;
}

void World::fill_out_default_marker(visualization_msgs::Marker &marker,
                                    uint8_t id,
                                    const Object &obj) {
	const Vec3 &given_coords = obj.coords;
	double const size = obj.radius;
	
	marker.header.frame_id = "map"; //"uav1/fcu"; //   uav1/local_origin;
	marker.header.stamp = ros::Time::now();
	marker.id = id;

    if (auto c = dynamic_cast<const Cylinder*>(&obj)) {
        marker.ns = "Cylinder";
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.scale.z = c->height;
    }
    else {
        marker.ns = "Sphere";
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.scale.z = size * 2;
    }

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
	marker.color.a = 0.7; // see-through or solid 0 to 1
	if (obj.is_goal()) {
		marker.color.r = 0.0;
		marker.color.g = 0.0;
		marker.color.b = 1.0;
	} else if (obj.is_start()) {
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
    } else {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    }
	marker.lifetime = ros::Duration(40);
}

void World::publish_path(const ros::Publisher &publisher, const std::vector<Vec3>& points, const std::string &number) {

    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "map"; //"uav1/fcu"; // ;
    line_strip.ns = "path" + number;
    line_strip.header.stamp = ros::Time::now();
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.id = 200;
    line_strip.scale.x = 0.1;

    Vec3 color = Vec3::random_vec3(0, 1, 0, 1, 0, 1);
    line_strip.color.r = (float) color.x;
    line_strip.color.g = (float) color.y;
    line_strip.color.b = (float) color.z;
    line_strip.color.a = 1;
    line_strip.pose.position.x = 0;
    line_strip.pose.position.y = 0;
    line_strip.pose.position.z = 0;
    for (const auto & point : points)
    {
        geometry_msgs::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
        line_strip.points.push_back(p);
    }
    line_strip.lifetime = ros::Duration(40);
    while (publisher.getNumSubscribers() < 1) {
        if (!ros::ok()) {
            std::cout << "Cannot publish, !ros::ok.\n";
        }
        ROS_WARN_ONCE("Waiting for at least one single sub.");
        sleep(1);
    }
    publisher.publish(line_strip);
}

void World::publish_trajectory(const ros::Publisher &publisher, const Trajectory &trajectory, const std::string &number) {

    ros::Rate rate(1000);

    std::vector<Vec3> points;
    visualization_msgs::Marker text_marker;
    text_marker.header.frame_id = "map"; //"uav1/fcu"; // ;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.id = 0;
    text_marker.scale.z = 0.1; // size of A letter
    text_marker.color.g = 1;
    text_marker.color.b = 1;
    text_marker.color.r = 1;
    text_marker.color.a = 1;

    for (const auto &time_point : trajectory.time_points) {
        // publish text
        points.push_back(time_point.first);
        text_marker.ns = number + ": " + std::to_string(time_point.second).substr(0, 5);
        text_marker.text = text_marker.ns;
        text_marker.header.stamp = ros::Time::now();
        text_marker.lifetime = ros::Duration(40);
        text_marker.id += 1;
        text_marker.pose.position.x = time_point.first.x;
        text_marker.pose.position.y = time_point.first.y;
        text_marker.pose.position.z = time_point.first.z + 0.12;

        while (publisher.getNumSubscribers() < 1) {
            if (!ros::ok()) {
                std::cout << "Cannot publish text, !ros::ok.\n";
            }
            ROS_WARN_ONCE("Waiting for at least one single sub.");
            sleep(1);
        }
        publisher.publish(text_marker);
        ros::spinOnce();
        rate.sleep();
    }

    publish_path(publisher, points, number);
}

void World::add_object(Object *newObj) {
    objects.emplace_back(newObj);
}

void World::add_obstacle(Object *newObj) {
    obstacles.emplace_back(newObj);
}
