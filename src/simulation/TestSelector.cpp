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
        case TEST_BUMPER:
            test_bumper();
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
    drones.emplace_back(false, 1, start1, goal1, goal_radius, drone_radius);
    drones.emplace_back(false, 2, start2, goal2, goal_radius, drone_radius);
    drones.emplace_back(false, 3, start3, goal3, goal_radius, drone_radius);
    drones.emplace_back(false, 4, start4, goal4, goal_radius, drone_radius);


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
    drones.emplace_back(false, 1, start1, goal1, goal_radius, drone_radius);
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

        if (drones[0].ready_map[ready_modules::ODOMETRY]) { // and drones[1].ready
                break;
        }
        ros::spinOnce();
        rate.sleep();
    }

    MotionMethods::go_through_a_trajectory(drones[0], drones[0].trajectory.trajectory_points, 0.5);
    // MotionMethods::go_through_a_trajectory(drones[1], drones[1].trajectory.trajectory_points, 0.5);

    std::cout << "End of the simulation." << std::endl;
}

void TestSelector::test_bumper() {

    ros::NodeHandle n;
    ros::Rate rate(10);

    // MARKER RVIZ
    ros::NodeHandle markers_node_publisher;
    ros::Publisher vis_pub = markers_node_publisher.advertise<visualization_msgs::Marker>
            ("visualization_marker", 10);

    ros::NodeHandle markers_array_node_publisher;
    ros::Publisher vis_array_pub = markers_array_node_publisher.advertise<visualization_msgs::MarkerArray>
            ("visualization_marker_array", 300);

    Drone drone(false, 1, Vec3(0,0,0), Vec3(5,0,0), 0.5, 0.5);

    std::cout << "Start\n";
    while (true) {
        if (drone.isReady()) {
            std::cout << "isReady() is true.\n";

            std::vector<double> sectors;
            sectors = drone.sectors_state->sectors;

            for (const auto &distance: sectors) {
                std::cout << distance << ", ";
            }
            std::cout << "end\n";
            Detection::update_obstacles_around_the_drone(drone);
            drone.world->publish_world(vis_array_pub);
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }
}



void TestSelector::one_drone_through_forest() {

    uint8_t drone_id = 14;
    bool real_world_test = false;

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

    double starting_goal_radius = 0.5;
    double drone_radius = 0.4;

    Vec3 start_local(0, 0, 0); // in local coordinates
    Vec3 goal_local(25, 0, 0);

    Drone drone(real_world_test, drone_id, start_local, goal_local, starting_goal_radius, drone_radius);

    Vec3 curr_pos_relative_to_local_0 = start_local;

    while (ros::ok()) {
        if (drone.ready_map.at(START)) { // Wait for the service
            break;
        }
        ros::spinOnce();
        std::cout << "Waiting for the service.\n";
        for (int i = 0; i < 20; ++i) {
            rate.sleep();
        }
    }

    std::cout << "********** Execution allowed. **********\n";

    while (!drone.isReady()) {
        ros::spinOnce();
        std::cout << "Odometry or bumper isn't ready.\n";
        for (int i = 0; i < 20; ++i) {
            rate.sleep();
        }
    }

    bool finished = false;
    while(!finished && ros::ok()) {

        std::cout << "Distance to goal=" << Vec3::distance_between_two_vec3(curr_pos_relative_to_local_0, goal_local) << ", goal_radius=" << starting_goal_radius << "\n";
        if (Vec3::distance_between_two_vec3(curr_pos_relative_to_local_0, goal_local) <= (starting_goal_radius)) {
            finished = true;
            std::cout << "Finished.\n";
            continue;
        }

        drone.goal_point = goal_local - curr_pos_relative_to_local_0; // Find path from the current point
        drone.start_point = Vec3(0,0,0);

        Detection::update_obstacles_around_the_drone(drone); // Locate obstacles around.

        std::cout << "\ncurr_pos_relative_to_local_0 = ";
        curr_pos_relative_to_local_0.printout();
        std::cout << "\n>>> Searching >>> \nStart: ";
        drone.start_point.printout();
        std::cout << "Goal: ";
        drone.goal_point.printout();
        drone.world->objects.clear();
        drone.world->add_object(new Sphere(drone.goal_radius, drone.goal_point));
        drone.world->add_object(new Sphere(drone.drone_radius, drone.start_point));
        auto k = drone.world->objects.size();
        drone.world->objects[k - 2]->set_as_a_goal();
        drone.world->objects[k - 1]->set_as_a_start();


        // TODO uncomment to have RViz (obstacles)
        drone.world->publish_world(vis_array_pub);


        std::cout << "Finding a path...\n";
        RRT_tree tree(drone.start_point, drone.world.get(), 3);
        drone.found_path = tree.find_path(RRTStarAlgorithm(),
                                              BinarySearchIntersection(),
                                              drone.goal_point,
                                              drone.goal_radius,
                                              drone.drone_radius);
        drone.trajectory = Trajectory(drone.found_path, 0.2, 0.25);
        RRT_tree::printout_the_path(drone.found_path);


        // TODO uncomment to have RViz (trajectory)
        drone.world->publish_trajectory(vis_pub, drone.trajectory, std::to_string(drone.uav_id));

        // Go through the first N points of a found trajectory
        const int N = 4;
        if (N < drone.trajectory.trajectory_points.size())
            drone.trajectory.trajectory_points.resize(N);

        std::cout << "Number of points to fly = " << drone.trajectory.trajectory_points.size() << ".\n";
        curr_pos_relative_to_local_0 = curr_pos_relative_to_local_0 + drone.trajectory.trajectory_points.back();
        std::cout << "Started flying.\n";
        MotionMethods::go_through_a_trajectory(drone, drone.trajectory.trajectory_points, 0.7);

        std::cout << "********* Finished cycle *******\n\n";

        ros::spinOnce();
        rate.sleep();
        for (int i = 0; i < 10; ++i) {
            rate.sleep();
        }
    }
}

