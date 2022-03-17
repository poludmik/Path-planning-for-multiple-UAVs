#include <ros/ros.h>
#include "simulation/TestSelector.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "drone_planner");

    TestSelector::run_simulation(ONE_DRONE_THROUGH_FOREST);
}
