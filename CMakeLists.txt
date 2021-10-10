cmake_minimum_required(VERSION 3.0.2)
project(drone_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_PREFIX_PATH "/opt/ros/noetic;${CMAKE_MODULE_PATH};")
#set(mrs_msgs_DIR "/home/misha/mrs_workspace/src/uav_core/ros_packages/mrs_msgs/msg")
set(mrs_msgs_DIR "/home/misha/mrs_workspace/devel/share/mrs_msgs/cmake")

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	geometry_msgs
	mrs_msgs
	message_generation
)

generate_messages(
	DEPENDENCIES
		std_msgs
		geometry_msgs
)

catkin_package(
	CATKIN_DEPENDS message_runtime
)

add_executable(planner src/main.cpp src/Vec3.cpp src/Vec3.h src/Object.cpp src/Object.h src/World.cpp src/World.h)

target_link_libraries(planner
	PRIVATE
		${catkin_LIBRARIES}
)

target_include_directories(planner
	PRIVATE
		${catkin_INCLUDE_DIRS}
)
