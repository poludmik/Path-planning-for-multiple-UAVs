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
#include "Sphere.h"
#include "Cylinder.h"

class Trajectory;

class World{
public:
    // Data Members
    std::vector<std::shared_ptr<Object>> objects;
    std::vector<std::shared_ptr<Object>> obstacles;

    void add_object(Object *newObj);

    void add_obstacle(Object *newObj);

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
