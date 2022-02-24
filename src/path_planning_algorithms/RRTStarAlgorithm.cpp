//
// Created by micha on 11/14/2021.
//

#include "RRTStarAlgorithm.h"
#include "../tree_structure/RRT_tree.h"
#include "../math/Vec3.h"
#include <algorithm>
#include <limits>

std::vector<Vec3> RRTStarAlgorithm::find_path_according_to_alg(const World *world_ptr,
                                                               const AvoidanceAlgorithm &avoid_alg,
                                                               Node *root,
                                                               const Vec3 &start_point,
                                                               const Vec3 &goal_point,
                                                               double goal_radius,
                                                               double neighbor_radius,
                                                               double droneRadius) const {

    int min_iters = 3000;

    Vec3 center = (start_point + goal_point) / 2.0;

    std::vector<std::shared_ptr<Node>> reached_goal_nodes;

    double dist_to_goal = Vec3::distance_between_two_vec3(start_point, goal_point);

    bool is_inside_an_obstacle = false;

    // if there is a straight line solution to the goal from the start
    //    for (const auto& obst : world_ptr->obstacles) {
    //        if (Vec3::DoesLineSegmentIntersectSphere(goal_point, start_point,
    //                                                 obst.coords, obst.radius)){
    //            is_inside_an_obstacle = true;
    //            break;
    //        }
    //    }
    //    if (!is_inside_an_obstacle) {
    //        auto closest = Node::find_the_closest_node(goal_point, root);
    //        closest->add_child(goal_point);
    //        return find_way_from_goal_to_root(closest->children.back().get());
    //    }


    // Main search
    size_t i = 0;
    while (true) {

        Vec3 rnd_point = Vec3::random_vec3(center.x - dist_to_goal, center.x + dist_to_goal,
                                           center.y - dist_to_goal, center.y + dist_to_goal,
                                           0.2,center.z + dist_to_goal);

        //center.z - 1, center.z + 1);
        is_inside_an_obstacle = false;

        auto closest = Node::find_the_closest_node(rnd_point, root);



        double distance_to_closest = Vec3::distance_between_two_vec3(rnd_point, closest->coords);
        if (distance_to_closest > 1.5) continue;

        std::vector<Node *> neighbors = Node::get_neighbors_in_radius(root, rnd_point, neighbor_radius);

        Node *best_neighbor = nullptr;
        double best_cost_to_new_node = closest->cost + distance_to_closest;
        double current_cost;

        for (const auto& neighbor:neighbors){

            is_inside_an_obstacle = false;

            current_cost = neighbor->cost + Vec3::distance_between_two_vec3(rnd_point, neighbor->coords);
            if (current_cost < best_cost_to_new_node) {

                // Check potential best neighbors
                for (const auto& obst : world_ptr->obstacles) {
                    if (avoid_alg.ThereIsIntersectionAlongThePath(neighbor->coords,rnd_point,
                                                                  obst.coords, droneRadius,
                                                                  obst.radius, obst.height)){
                        is_inside_an_obstacle = true;
                        break;
                    }
                }
                if (is_inside_an_obstacle) continue;

                best_cost_to_new_node = current_cost;
                best_neighbor = neighbor;
            }
        }

        is_inside_an_obstacle = false;


        std::shared_ptr<Node> new_node;
        if (best_neighbor != nullptr) {
            best_neighbor->add_child(rnd_point);
            new_node = best_neighbor->children.back();
        } else {
            // Check closest
            for (const auto& obst : world_ptr->obstacles) {
                if (avoid_alg.ThereIsIntersectionAlongThePath(closest->coords,rnd_point,
                                                              obst.coords, droneRadius,
                                                              obst.radius, obst.height)){
                    is_inside_an_obstacle = true;
                    break;
                }
            }
            if (is_inside_an_obstacle) continue;

            closest->add_child(rnd_point);
            new_node = closest->children.back();
        }


        for (auto& neighbor:neighbors){
            Node *parent = neighbor->parent;
            is_inside_an_obstacle = false;
            if (new_node->cost + Vec3::distance_between_two_vec3(rnd_point, neighbor->coords) < neighbor->cost){

                // Check neighbor and new_node/rnd_point
                for (const auto& obst : world_ptr->obstacles) {
                    if (avoid_alg.ThereIsIntersectionAlongThePath(neighbor->coords,rnd_point,
                                                                  obst.coords, droneRadius,
                                                                  obst.radius, obst.height)){
                        is_inside_an_obstacle = true;
                        break;
                    }
                }
                if (is_inside_an_obstacle) continue;

                neighbor->change_parent(new_node.get());
            }
        }

//        is_inside_an_obstacle = false;
//
//        // if there is a straight line solution to the goal from newly added point
//        //        for (const auto& obst : world_ptr->obstacles) {
//        //            if (Vec3::DoesLineSegmentIntersectSphere(goal_point, rnd_point,
//        //                                                     obst.coords, obst.radius)){
//        //                is_inside_an_obstacle = true;
//        //                break;
//        //            }
//        //        }
//        //        if (is_inside_an_obstacle) {
//        //            if (Vec3::distance_between_two_vec3(rnd_point, goal_point) < goal_radius) {
//        //                return find_way_from_goal_to_root(closest->children.back().get());
//        //            }
//        //        }
//        //        else {
//        //            closest->children.back()->add_child(goal_point);
//        //            return find_way_from_goal_to_root(closest->children.back()->children.back().get());
//        //        }
//
        ++i;

        neighbors.clear();

        if (i % 1000 == 0) std::cout << i << "\n";

        if (Vec3::distance_between_two_vec3(rnd_point, goal_point) < goal_radius) {

            if (best_neighbor != nullptr) {
//                std::cout <<"a\n";

                reached_goal_nodes.push_back(best_neighbor->children.back());
                Node *best_in_goal = best_neighbor->children.back().get();

                double min_distance = std::numeric_limits<double>::max();
                for (const auto &x : reached_goal_nodes){
                    if (x->cost < min_distance) {
                        min_distance = x->cost;
                        best_in_goal = x.get();
                    }
                }

                if (i < min_iters) continue;

                return Algorithm::find_way_from_goal_to_root(best_in_goal);
            }
//            std::cout <<"b\n";

            if (i < min_iters) continue;
            return Algorithm::find_way_from_goal_to_root(closest->children.back().get());
        }
    }
}



////// TEST FIND NEIGHBORS
//    Vec3 coords(0, 0 ,0);
//    Node zero(coords);
//    root = &zero;
//    Vec3 coords1(-1, 0 ,0);
//    root->add_child(coords1);
//    Vec3 coords2(1, 0 ,0);
//    root->add_child(coords2);
//    Vec3 coords3(2, 0 ,0);
//    root->children.back()->add_child(coords3);
//
//    Vec3 coordsLast(2, 1, 0);
//    Node reference(coordsLast);
//
//    std::vector<std::shared_ptr<Node>> array = get_neighbors_in_radius(root, &reference, 1.5);
//
//    for (const auto& x:array){
//        std::cout << x->coords.x << " " << x->coords.y << "\n";
//    }
//
//    std::vector<Vec3> a;
//
//    return a;
