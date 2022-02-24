//
// Created by micha on 2/17/2022.
//

#include "Trajectory.h"
#include "Drone.h"

void Trajectory::equally_divide_path_in_time(const std::vector<Vec3> &path) {

    std::vector<std::pair<Vec3, double>> trajectory;
    std::pair<Vec3, double> point_in_time;

    double global_time_shift = 0.0;

    for (uint i = 0; i < path.size() - 1; ++i){
        Vec3 current_point = path[i];
        Vec3 next_point = path[i + 1];
        Vec3 direction = next_point - current_point;

        double distance = Vec3::distance_between_two_vec3(current_point, next_point);

        point_in_time.first = current_point;
        point_in_time.second = global_time_shift;
        time_points.push_back(point_in_time);

        trajectory_points.push_back(current_point);

        double current_dist_shift = ds_between_points;

        while (current_dist_shift < distance) {
            Vec3 between_point(current_point.x + direction.x * current_dist_shift / distance,
                               current_point.y + direction.y * current_dist_shift / distance,
                               current_point.z + direction.z * current_dist_shift / distance);


            if (Vec3::distance_between_two_vec3(between_point, next_point) > ds_between_points / 3) {

                global_time_shift += dt_between_points;
                current_dist_shift += ds_between_points;

                point_in_time.first = between_point;
                point_in_time.second = global_time_shift;

                time_points.push_back(point_in_time);
                trajectory_points.push_back(between_point);
            } else {
                break;
            }
        }

        global_time_shift += dt_between_points;

    }

    point_in_time.first = path.back();
    point_in_time.second = global_time_shift;
    time_points.push_back(point_in_time);
    trajectory_points.push_back(path.back());

}

Trajectory::Trajectory(const std::vector<Vec3> &path, const double dt, const double ds) {
    dt_between_points = dt;
    ds_between_points = ds;
    equally_divide_path_in_time(path);
}

std::pair<std::vector<Vec3>, int> Trajectory::find_intersects_of_two_trajectories(const Trajectory &priority_traj,
                                                                  const Trajectory &secondary_traj,
                                                                  double radius1,
                                                                  double radius2) {

    std::vector<Vec3> intersection_points;

    double time_epsilon = 0.001;
    bool found_intersection = false;
    int last_good = -1;
    int idx = 0;

    for (const auto &second_time_point : secondary_traj.time_points) {

        for (const auto &first_time_point : priority_traj.time_points) {

            if (first_time_point.second < second_time_point.second + time_epsilon &&
            first_time_point.second > second_time_point.second - time_epsilon){
                if (AvoidanceAlgorithm::DoesSphereIntersectSphere(first_time_point.first,
                                                                  second_time_point.first,
                                                                  radius1,
                                                                  radius2)) {
                    intersection_points.push_back(first_time_point.first);
                    found_intersection = true;

                }
            }
        }

        if (!found_intersection) {
            last_good = idx;
        }
        ++idx;

    }

    std::pair<std::vector<Vec3>, int> returned_value;
    returned_value.first = intersection_points;
    returned_value.second = last_good; // last point on second trajectory without an intersection

    return returned_value;
}

void Trajectory::resolve_all_conflicts_with_new_trajectories(World &global_world,
                                                           std::vector<Drone> &drones) {

    for (int i = 1; i < drones.size(); ++i) {

        World local_world = global_world;
        std::vector<Vec3> intersects_with_one;
        std::pair<std::vector<Vec3>, int> intersects_and_last_good;

        bool zero_conflicts = false;

        while (!zero_conflicts) {

            zero_conflicts = true;

            unsigned long min_last_good = drones[i].trajectory.time_points.size() - 1;

            for (int j = 0; j < drones.size(); ++j) {

                if (j == i) continue;

                intersects_and_last_good = find_intersects_of_two_trajectories(drones[j].trajectory,
                                                                               drones[i].trajectory,
                                                                               drones[j].drone_radius,
                                                                               drones[i].drone_radius);

                intersects_with_one = intersects_and_last_good.first;
                if (intersects_and_last_good.second < min_last_good) {
                    min_last_good = intersects_and_last_good.second;
                }

                std::cout << "All the obstacles:\n";
                for (const auto &point: intersects_with_one) {
                    point.printout();
                    zero_conflicts = false;
                    local_world.add_obstacle("sphere", drones[j].drone_radius, point);
                    //global_world.add_object("sphere", drones[j].drone_radius, point);
                }
            }

            for (const Drone &drone: drones) {
                if (drone.goal_point == drones[i].goal_point) continue;
                local_world.add_obstacle("sphere", drone.goal_radius, drone.goal_point);
                local_world.add_obstacle("sphere", drone.goal_radius, drone.start_point);
            }

            std::cout << "\nmin_last_good: " << min_last_good << "\n\n";

            if (min_last_good == -1) {
                // no collisions
                std::cout << "No collisions.\n";
                continue;
            }

            while (min_last_good > 0) {
                bool flag = false;
                Vec3 start = drones[i].trajectory.time_points[min_last_good].first;
                for (auto const &x: local_world.obstacles) {

                    if (AvoidanceAlgorithm::DoesSphereIntersectSphere(x.coords,
                                                                      start,
                                                                      x.radius,
                                                                      drones[i].drone_radius)) {
                        flag = true;
                        break;
                    }
                }
                if (flag) {
                    --min_last_good;
                    continue;
                } else {
                    break;
                }
            }

            Vec3 new_starting_point = drones[i].trajectory.time_points[min_last_good].first;
            std::cout << "new_starting_point: ";
            new_starting_point.printout();
            std::cout << "min_last_good: " << min_last_good << "\n\n";

            for (auto const &x: local_world.obstacles) {
                if (AvoidanceAlgorithm::DoesSphereIntersectSphere(x.coords,
                                                                  drones[i].trajectory.time_points[min_last_good].first,
                                                                  x.radius,
                                                                  drones[i].drone_radius)) {

                    std::cout << "The start point is occupied, error.\n";
                    std::cout << "Obstacle:";
                    x.coords.printout();
                    break;
                }
            }



            double neighbor_radius = 2;
            RRT_tree tree(new_starting_point, &local_world, neighbor_radius);
            std::vector<Vec3> path_from_new_start = tree.find_path(RRTStarAlgorithm(),
                                                                   BinarySearchIntersection(),
                                                                   drones[i].goal_point,
                                                                   drones[1].goal_radius,
                                                                   drones[1].drone_radius);

            std::cout << "here" << "\n\n";

            // found_path concatenate
            drones[i].trajectory.trajectory_points.resize(min_last_good);
            drones[i].found_path = drones[i].trajectory.trajectory_points;

            drones[i].found_path.insert(drones[i].found_path.end(), path_from_new_start.begin(),
                                        path_from_new_start.end());

            drones[i].trajectory = Trajectory(drones[i].found_path, 0.2, 0.3);
        }
    }

}
