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

class Object;
class Trajectory;

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
                                          const std::string &number);

    World()=default;
    
    ~World();

private:

    static void fill_out_default_marker(visualization_msgs::Marker& marker,
                                        uint8_t id,
                                        const Object &obj);
};
