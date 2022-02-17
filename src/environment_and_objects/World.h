//
// Created by misha on 10/10/21.
//

#pragma once


#include <string>
#include <ros/ros.h>
#include "../math/Vec3.h"
#include <vector>
#include <mrs_msgs/UavState.h>
#include <mrs_msgs/VelocityReferenceStamped.h>
#include <visualization_msgs/Marker.h>
#include "../motion/Trajectory.h"

class Object;

class World{

public:
    
    // Data Members
    std::vector<Object> objects;
    std::vector<Object> obstacles;
    
    void add_object(const std::string &type, double radius, const Vec3 &given_coords);
    
    void add_object(double radius, const Vec3 &given_coords);

    void add_obstacle(const std::string &type, double radius, const Vec3 &given_coords);

    void add_obstacle(const std::string &type, double radius, const Vec3 &given_coords, double height);

    void add_obstacle(double radius, const Vec3 &given_coords);
    
    void publish_world(const ros::Publisher &publisher) const;

    static void publish_path(const ros::Publisher &publisher, const std::vector<Vec3>& points, const std::string &number);

    static void publish_trajectory(const ros::Publisher &publisher,
                                          const Trajectory &trajectory,
                                          const std::string &number) {

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


    World()=default;
    
    ~World();

private:

    static void fill_out_default_marker(visualization_msgs::Marker& marker,
                                        uint8_t id,
                                        const Object &obj);
};
