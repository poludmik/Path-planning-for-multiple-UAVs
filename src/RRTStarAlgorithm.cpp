//
// Created by micha on 11/14/2021.
//

#include "RRTStarAlgorithm.h"
#include "RRT_tree.h"
#include "Vec3.h"
#include <algorithm>

std::vector<Vec3>
RRTStarAlgorithm::find_path_according_to_alg(const World *world_ptr,
                                             Node *root,
                                             const Vec3 &start_point,
                                             const Vec3 &goal_point,
                                             double goal_radius,
                                             double neighbor_radius) const {

    Vec3 center = (start_point + goal_point) / 2.0;

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
                                           0, 0);

        is_inside_an_obstacle = false;

        auto closest = Node::find_the_closest_node(rnd_point, root);

        double distance_to_closest = Vec3::distance_between_two_vec3(rnd_point, closest->coords);
        if (distance_to_closest > 1.0) continue;

////        for (const auto& obst : world_ptr->obstacles) {
////            if (Vec3::DoesLineSegmentIntersectSphere(closest->coords,rnd_point,
////                                                     obst.coords, obst.radius)){
////                is_inside_an_obstacle = true;
////                break;
////            }
////        }
////        if (is_inside_an_obstacle) continue;

        std::vector<Node *> neighbors = Node::get_neighbors_in_radius(root, rnd_point, neighbor_radius);

        Node *best_neighbor = nullptr;
        double best_cost_to_new_node = closest->cost + distance_to_closest;
        double current_cost;

        for (const auto& neighbor:neighbors){

            is_inside_an_obstacle = false;

            current_cost = neighbor->cost + Vec3::distance_between_two_vec3(rnd_point, neighbor->coords);
            if (current_cost < best_cost_to_new_node) {

////                for (const auto& obst : world_ptr->obstacles) {
////                    if (Vec3::DoesLineSegmentIntersectSphere(neighbor->coords,rnd_point,
////                                                             obst.coords, obst.radius)){
////                        is_inside_an_obstacle = true;
////                        break;
////                    }
////                }
////                if (is_inside_an_obstacle) continue;

                best_cost_to_new_node = current_cost;
                best_neighbor = neighbor;
            }
        }

        is_inside_an_obstacle = false;

        std::shared_ptr<Node> new_node;
        if (best_neighbor != nullptr) {
            best_neighbor->add_child(rnd_point);
            //delete parent from neighbors
//            for (auto x = neighbors.begin(); x < neighbors.end(); ++x){
//                if ((*x) == best_neighbor) {
//                    neighbors.erase(x);
//                    break;
//                }
//            }
            new_node = best_neighbor->children.back();
        } else {
            closest->add_child(rnd_point);
//            for (auto x = neighbors.begin(); x < neighbors.end(); ++x){
//                if (*x == closest) {
//                    neighbors.erase(x);
//                    break;
//                }
//            }
            new_node = closest->children.back();
        }


        for (auto& neighbor:neighbors){

            Node *parent = neighbor->parent;

//            is_inside_an_obstacle = false;
            if (new_node->cost + Vec3::distance_between_two_vec3(rnd_point, neighbor->coords) < neighbor->cost){


                std::cout << "----------------------\nstart:\n";
                for (auto x = parent->children.begin(); x < parent->children.end(); ++x) {
                    std::cout << (*x)->cost << " ";
                }
                std::cout << "\n";

                neighbor->change_parent(new_node.get());

                std::cout << "\nend:\n";
                for (auto x = parent->children.begin(); x < parent->children.end(); ++x) {
                    std::cout << (*x)->cost << " ";
                }
                std::cout << "\n-----------------\n\n";
//                break; //TODO

//                new_node->add_child(neighbor->coords);
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

        if (i < 10000) continue;

        if (Vec3::distance_between_two_vec3(rnd_point, goal_point) < goal_radius) {


            if  (best_neighbor != nullptr) {
                std::cout <<"a\n";
                return Algorithm::find_way_from_goal_to_root(best_neighbor->children.back().get());
            }
            std::cout <<"b\n";
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