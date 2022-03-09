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

include(FetchContent)

FetchContent_Declare(json
		GIT_REPOSITORY https://github.com/ArthurSonzogni/nlohmann_json_cmake_fetchcontent
		GIT_TAG v3.10.4)

FetchContent_GetProperties(json)
if(NOT json_POPULATED)
	FetchContent_Populate(json)
	add_subdirectory(${json_SOURCE_DIR} ${json_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()


add_executable(planner src/main.cpp
		src/math/Vec3.cpp
		src/math/Vec3.h
		src/environment_and_objects/Object.cpp
		src/environment_and_objects/Object.h
		src/environment_and_objects/World.cpp
		src/environment_and_objects/World.h
		src/tree_structure/Node.cpp
		src/tree_structure/Node.h
		src/tree_structure/RRT_tree.cpp
		src/tree_structure/RRT_tree.h
        src/path_planning_algorithms/Algorithm.h
		src/path_planning_algorithms/RRTStarAlgorithm.cpp
		src/path_planning_algorithms/RRTStarAlgorithm.h
		src/path_planning_algorithms/RRTAlgorithm.cpp
		src/path_planning_algorithms/RRTAlgorithm.h
		src/avoidance/AvoidanceAlgorithm.h
		src/avoidance/BinarySearchIntersection.cpp
		src/avoidance/BinarySearchIntersection.h
		src/avoidance/LinearAlgebraIntersection.cpp
		src/avoidance/LinearAlgebraIntersection.h
		src/motion/MotionMethods.cpp
		src/motion/MotionMethods.h
		src/motion/Drone.cpp
		src/motion/Drone.h
		src/motion/Trajectory.cpp
		src/motion/Trajectory.h
		src/2D_plot/Plot2D.cpp
		src/2D_plot/Plot2D.h
		src/environment_and_objects/Sphere.h
		src/environment_and_objects/Cylinder.h
		src/simulation/TestSelector.cpp src/simulation/TestSelector.h)

target_link_libraries(planner PRIVATE nlohmann_json::nlohmann_json)

target_link_libraries(planner
	PRIVATE
		${catkin_LIBRARIES}
)

target_include_directories(planner
	PRIVATE
		${catkin_INCLUDE_DIRS}
)
