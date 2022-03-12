//
// Created by micha on 3/9/2022.
//

#include "TestSelector.h"

void TestSelector::run_simulation(const TestCase test_case) {
    switch(test_case) {
        case GO_THROUGH_TRAJECTORIES:
            fly_through_found_paths();
            break;
        case FIND_TRAJECTORIES:
            basic_trajectory_search();
            break;
        case ONE_DRONE_THROUGH_FOREST:
            one_drone_through_forest();
            break;
        default:
            std::cout << "The simulation under the given name hasn't been found." << std::endl;
    }
}

void TestSelector::basic_trajectory_search() {

    // VELOCITY CONTROL
    size_t uav_id = 1;
    std::string vel_pub_topic  = "/uav" + std::to_string(uav_id) + "/control_manager/velocity_reference";

    ros::NodeHandle n;
    ros::Publisher  vel_pub   = n.advertise<mrs_msgs::VelocityReferenceStamped>(vel_pub_topic, 100);

    ros::Rate rate(10);

    // MARKER RVIZ
    ros::NodeHandle markers_node_publisher;
    ros::Publisher vis_pub = markers_node_publisher.advertise<visualization_msgs::Marker>
            ("visualization_marker", 10);

    ros::NodeHandle markers_array_node_publisher;
    ros::Publisher vis_array_pub = markers_array_node_publisher.advertise<visualization_msgs::MarkerArray>
            ("visualization_marker_array", 300);

    World my_world("map");

    double goal_radius = 0.5;
    double drone_radius = 0.5;

    Vec3 start1(-4.5, 2, 1);
    Vec3 goal1(3.5, 2, 1);

    Vec3 start2(-4.5, 0.8, 1);
    Vec3 goal2(3.5, -0.8, 1);

    Vec3 start3(-4.5, -0.8, 1);
    Vec3 goal3(3.5, 0.8,1);

    Vec3 start4(-4.5, -2.3, 1);
    Vec3 goal4(3.5, -2.3, 1);


    std::vector<Drone> drones;
    drones.emplace_back(1, start1, goal1, goal_radius, drone_radius);
    drones.emplace_back(2, start2, goal2, goal_radius, drone_radius);
    drones.emplace_back(3, start3, goal3, goal_radius, drone_radius);
    drones.emplace_back(4, start4, goal4, goal_radius, drone_radius);


    Vec3 standing_center(0, 0, 0);
    my_world.add_obstacle(new Cylinder(0.4, standing_center, 3.5));
    Vec3 standing_center1(0, 2, 0);
    my_world.add_obstacle(new Cylinder(0.3, standing_center1, 4));
    Vec3 standing_center2(0, -2.5, 0);
    my_world.add_obstacle(new Cylinder(0.35, standing_center2, 3));
    Vec3 standing_center6(0, -3.5, 0);
    my_world.add_obstacle(new Cylinder(0.35, standing_center6, 3));
    Vec3 standing_center7(0, -4.5, 0);
    my_world.add_obstacle(new Cylinder(0.35, standing_center7, 3));
    Vec3 standing_center4(-2, 0, 0);
    my_world.add_obstacle(new Cylinder(0.4, standing_center4, 2));
    Vec3 standing_center5(2, 1, 0);
    my_world.add_obstacle(new Cylinder(0.4, standing_center5, 5));
    Vec3 standing_center8(2, -1, 0);
    my_world.add_obstacle(new Cylinder(0.2, standing_center8, 3.5));
    Vec3 standing_center9(-2, -2, 0);
    my_world.add_obstacle(new Cylinder(0.4, standing_center9, 1.5));
    Vec3 standing_center10(0, 3, 0);
    my_world.add_obstacle(new Cylinder(0.35, standing_center10, 3));
    Vec3 standing_center11(0, 4, 0);
    my_world.add_obstacle(new Cylinder(0.35, standing_center11, 3));
    Vec3 standing_center12(0, 5, 0);
    my_world.add_obstacle(new Cylinder(0.35, standing_center12, 3));
    Vec3 standing_center13(-3, 1, 0);
    my_world.add_obstacle(new Cylinder(0.6, standing_center13, 2.5));
    Vec3 standing_center666(0, 1, 2.5);
    my_world.add_obstacle(new Cylinder(1, standing_center666, 0.7));
    Vec3 standing_center667(0, 1, 0);
    my_world.add_obstacle(new Cylinder(1, standing_center667, 0.7));


    bool performance_mode = true;
    // performance mode is better for 3D, it is less constraining
    Trajectory::find_trajectories_without_time_collisions(my_world, drones, performance_mode);

    for (Drone &drone : drones) {

        for (const auto &point_in_time: drone.trajectory.time_points) {
            my_world.add_object(new Sphere(0.05, point_in_time.first));
        }

        my_world.add_object(new Sphere(drone.goal_radius, drone.goal_point));
        my_world.add_object(new Sphere(drone.drone_radius, drone.start_point));
        auto k = my_world.objects.size();
        my_world.objects[k - 2]->set_as_a_goal();
        my_world.objects[k - 1]->set_as_a_start();

        my_world.publish_trajectory(vis_pub, drone.trajectory, std::to_string(drone.uav_id));

        ros::spinOnce();
        rate.sleep();
    }

    std::string file_name = "Multiple_trajectories.json";
    std::string plot_title = "Multiple paths";
    Plot2D::write_multiple_trajectories_to_json_file(drones, file_name, my_world.obstacles, plot_title);

    my_world.publish_world(vis_array_pub);

    std::cout << "End of the simulation." << std::endl;
}

void TestSelector::fly_through_found_paths() {

    // VELOCITY CONTROL
    size_t uav_id = 1;
    std::string vel_pub_topic  = "/uav" + std::to_string(uav_id) + "/control_manager/velocity_reference";

    ros::NodeHandle n;
    ros::Publisher  vel_pub   = n.advertise<mrs_msgs::VelocityReferenceStamped>(vel_pub_topic, 100);

    ros::Rate rate(10);

    // MARKER RVIZ
    ros::NodeHandle markers_node_publisher;
    ros::Publisher vis_pub = markers_node_publisher.advertise<visualization_msgs::Marker>
            ("visualization_marker", 10);

    ros::NodeHandle markers_array_node_publisher;
    ros::Publisher vis_array_pub = markers_array_node_publisher.advertise<visualization_msgs::MarkerArray>
            ("visualization_marker_array", 300);

    World my_world("uav1/fcu");

    double goal_radius = 0.2;
    double drone_radius = 0.5;

    Vec3 start1(0, 0, 0); // in local coordinates
    Vec3 goal1(4, 0, 0);

    Vec3 start2(-4.5, 0.8, 1);
    Vec3 goal2(3.5, -0.8, 1);

    std::vector<Drone> drones;
    drones.emplace_back(1, start1, goal1, goal_radius, drone_radius);
    // drones.emplace_back(2, start2, goal2, goal_radius, drone_radius);

    Vec3 standing_center(2, 0, -1);
    my_world.add_obstacle(new Cylinder(0.4, standing_center, 3.5));

    bool performance_mode = true;
    // performance mode is better for 3D, it is less constraining
    Trajectory::find_trajectories_without_time_collisions(my_world, drones, performance_mode);

    for (Drone &drone : drones) {

        for (const auto &point_in_time: drone.trajectory.time_points) {
            my_world.add_object(new Sphere(0.05, point_in_time.first));
        }

        my_world.add_object(new Sphere(drone.goal_radius, drone.goal_point));
        my_world.add_object(new Sphere(drone.drone_radius, drone.start_point));
        auto k = my_world.objects.size();
        my_world.objects[k - 2]->set_as_a_goal();
        my_world.objects[k - 1]->set_as_a_start();

        my_world.publish_trajectory(vis_pub, drone.trajectory, std::to_string(drone.uav_id));

        ros::spinOnce();
        rate.sleep();
    }

    my_world.publish_world(vis_array_pub);

    while (ros::ok()) {

        if (drones[0].ready) { // and drones[1].ready
                break;
        }
        ros::spinOnce();
        rate.sleep();
    }

    MotionMethods::go_through_a_trajectory(drones[0], drones[0].trajectory.trajectory_points, 0.5);
    // MotionMethods::go_through_a_trajectory(drones[1], drones[1].trajectory.trajectory_points, 0.5);

    std::cout << "End of the simulation." << std::endl;
}


void TestSelector::one_drone_through_forest() {

    // VELOCITY CONTROL
    size_t uav_id = 1;
    std::string vel_pub_topic = "/uav" + std::to_string(uav_id) + "/control_manager/velocity_reference";

    ros::NodeHandle n;
    ros::Rate rate(10);
    mrs_msgs::UavState::ConstPtr cur_uav_state;


    // MARKER RVIZ
    ros::NodeHandle markers_node_publisher;
    ros::Publisher vis_pub = markers_node_publisher.advertise<visualization_msgs::Marker>
            ("visualization_marker", 10);

    ros::NodeHandle markers_array_node_publisher;
    ros::Publisher vis_array_pub = markers_array_node_publisher.advertise<visualization_msgs::MarkerArray>
            ("visualization_marker_array", 300);

    World my_world("uav1/fcu");

    double goal_radius = 0.5;
    double drone_radius = 0.3;

    Vec3 start_local(0, 0, 0); // in local coordinates
    Vec3 goal_local(5, 0, 0);

    std::vector<Drone> drones;
    drones.emplace_back(1, start_local, goal_local, goal_radius, drone_radius);

    Vec3 curr_pos_odom;
    Vec3 init_pos_odom;
    Vec3 curr_pos_relative_to_local_0 = start_local;

    while (ros::ok()) {
        if (drones[0].ready) {
            cur_uav_state = drones[0].uav_state;
            curr_pos_odom = Vec3(cur_uav_state->pose.position.x,
                                 cur_uav_state->pose.position.y,
                                 cur_uav_state->pose.position.z);
            init_pos_odom = curr_pos_odom;
            break;
        }
    }

    bool finished = false;

    while(!finished && ros::ok()) {

        cur_uav_state = drones[0].uav_state;

        curr_pos_odom = Vec3(cur_uav_state->pose.position.x,
                             cur_uav_state->pose.position.y,
                             cur_uav_state->pose.position.z);

        if (Vec3::distance_between_two_vec3(curr_pos_relative_to_local_0, goal_local) <= goal_radius) {
            finished = true;
            continue;
        }


        // TODO, locate obstacles around.


        // TODO, add obstacles to the local world.
        World local_world_relative_to_uav = my_world;
        for (const auto &obstacle : obstacles) {
            local_world_relative_to_uav.add_obstacle(new Cylinder(0.4, obstacle.center, 3.5));
        }


        // Find path from the current point
        drones[0].goal_point = drones[0].goal_point - curr_pos_relative_to_local_0;
        drones[0].start_point = Vec3(0,0,0);
        Trajectory::find_trajectories_without_time_collisions(local_world_relative_to_uav, drones, false);


        // Go through the first N points
        const int N = 5;
        drones[0].trajectory.trajectory_points.resize(N);
        curr_pos_relative_to_local_0 = curr_pos_relative_to_local_0 + drones[0].trajectory.trajectory_points.back();
        MotionMethods::go_through_a_trajectory(drones[0], drones[0].trajectory.trajectory_points, 0.2);

        ros::spinOnce();
        rate.sleep();
    }
}
