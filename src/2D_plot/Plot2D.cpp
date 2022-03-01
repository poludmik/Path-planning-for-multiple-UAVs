//
// Created by micha on 2/25/2022.
//

#include "Plot2D.h"

void Plot2D::write_tree_structure_to_json_file(Node *root,
                                               const std::string &tree_name,
                                               const std::string &filename,
                                               const std::vector<Vec3> &path,
                                               const Vec3 &goal,
                                               const double goal_radius,
                                               const std::vector<std::shared_ptr<Object>> &obstacles) {
    std::queue<Node *> my_queue;

    my_queue.push(root);

    nlohmann::json j_structure;

    std::ofstream o(filename);

    size_t idx = 1;
    while (!my_queue.empty()) {
        // front() returns reference, after that we are calling pop,
        // so we need to dereference and take a new pointer to prevent invalid reference
        auto current = &(*my_queue.front());
        my_queue.pop();

        for (const auto &i: current->children) {
            std::vector<double> line_sector;

            line_sector.push_back(current->coords.x);
            line_sector.push_back(current->coords.y);
            line_sector.push_back(current->coords.z);

            line_sector.push_back(i->coords.x);
            line_sector.push_back(i->coords.y);
            line_sector.push_back(i->coords.z);

            j_structure["tree" + std::to_string(idx)] = line_sector;

            line_sector.clear();

            my_queue.push(i.get());
            ++idx;
        }
    }
    j_structure["tree_size"] = idx;

    j_structure["start"] = {root->coords.x, root->coords.y, root->coords.z};
    j_structure["goal"] = {goal.x, goal.y, goal.z};
    idx = 1;
    for (const auto &i : path){
        std::vector<double> path_point;

        path_point.push_back(i.x);
        path_point.push_back(i.y);
        path_point.push_back(i.z);

        j_structure["path" + std::to_string(idx)] = path_point;
        path_point.clear();
        ++idx;
    }
    j_structure["path_size"] = idx;

    j_structure["goal_radius"] = goal_radius;

    // Write obstacles
    write_obstacles_to_json_file(j_structure, obstacles);

    j_structure["graph_title"] = tree_name;

    o << std::setw(8) << j_structure << std::endl;
}


void Plot2D::write_multiple_trajectories_to_json_file(const std::vector<Drone> &drones,
                                                      const std::string &filename,
                                                      const std::vector<std::shared_ptr<Object>> &obstacles,
                                                      const std::string &graph_title) {
    nlohmann::json j_structure;

    std::ofstream o(filename);

    for (const Drone &drone : drones) {

        unsigned long path_size = drone.trajectory.trajectory_points.size();

        std::string drone_id = std::to_string(drone.uav_id);

        j_structure["path_size_of_uav" + drone_id] = path_size;

        j_structure["start_of_uav" + drone_id] = {drone.start_point.x, drone.start_point.y, drone.start_point.z};

        j_structure["goal_of_uav" + drone_id] = {drone.goal_point.x, drone.goal_point.y, drone.goal_point.z};

        j_structure["uav" + drone_id + "_radius"] = drone.drone_radius;

        j_structure["uav" + drone_id + "_goal_radius"] = drone.goal_radius;

        unsigned int idx = 0;
        for (const Vec3 & path_point : drone.trajectory.trajectory_points) {
            j_structure["uav" + drone_id + "_path_point" + std::to_string(idx)] = { path_point.x,
                                                                                        path_point.y,
                                                                                        path_point.z };
            ++idx;
        }

    }

    write_obstacles_to_json_file(j_structure, obstacles);

    j_structure["graph_title"] = graph_title;

    j_structure["number_of_drones"] = drones.size();

    o << std::setw(8) << j_structure << std::endl;
}


void Plot2D::write_obstacles_to_json_file(nlohmann::json &j_structure, const std::vector<std::shared_ptr<Object>> &obstacles) {

    unsigned int idx = 0;

    for (const auto &p : obstacles){
        const auto &x = *p;
        j_structure["obstacle" + std::to_string(idx)] = {x.radius, x.coords.x, x.coords.y, x.coords.z};
        ++idx;
    }

    j_structure["obstacles_number"] = idx;
}
