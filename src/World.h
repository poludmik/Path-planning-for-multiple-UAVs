//
// Created by misha on 10/10/21.
//

#pragma once


#include <string>
#include <ros/ros.h>
#include "Vec3.h"
#include <vector>
#include <mrs_msgs/UavState.h>
#include <mrs_msgs/VelocityReferenceStamped.h>
#include <visualization_msgs/Marker.h>

class Object;

class World{
public:
    
    // Data Members
    std::vector<Object> obstacles;
    
    void add_object(const std::string &name, double radius, const Vec3 &given_coords);
    
    void add_object(double radius, const Vec3 &given_coords);
    
    void print_out_objects();
    
    void publish_world(ros::Publisher &publisher);
    
    World()=default;
    
    ~World();;

private:
    
    static void fill_out_default_marker(visualization_msgs::Marker& marker,
                                        uint8_t id,
                                        const Object &obj);
};
