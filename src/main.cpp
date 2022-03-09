#include <ros/ros.h>
#include "simulation/TestSelector.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "drone_planner");

    TestSelector::run_simulation(FIND_TRAJECTORIES);
}
