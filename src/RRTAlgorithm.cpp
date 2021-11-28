//
// Created by micha on 11/14/2021.
//

#include "RRTAlgorithm.h"
#include "RRT_tree.h"

std::vector<Vec3> RRTAlgorithm::find_path_according_to_alg(const World *world_ptr,
                                                           Node *root,
                                                           const Vec3 &start_point,
                                                           const Vec3 &goal_point,
                                                           double goal_radius,
                                                           double neighbor_radius) const {

//    RRT_tree tree(start_point, world_ptr, neighbor_radius);

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
    while (true) {
        Vec3 rnd_point = Vec3::random_vec3(center.x - dist_to_goal, center.x + dist_to_goal,
                                           center.y - dist_to_goal, center.y + dist_to_goal,
                                           0, 0);

        is_inside_an_obstacle = false;

        auto closest = Node::find_the_closest_node(rnd_point, root);

        if (Vec3::distance_between_two_vec3(rnd_point, closest->coords) > 1.5) continue;

        for (const auto& obst : world_ptr->obstacles) {
            if (Vec3::DoesLineSegmentIntersectSphere(closest->coords,rnd_point,
                                                     obst.coords, obst.radius)){
                is_inside_an_obstacle = true;
                break;
            }
        }
        if (is_inside_an_obstacle) continue;
        is_inside_an_obstacle = false;

        closest->add_child(rnd_point);

        // if there is a straight line solution to the goal from newly added point
//        for (const auto& obst : world_ptr->obstacles) {
//            if (Vec3::DoesLineSegmentIntersectSphere(goal_point, rnd_point,
//                                                     obst.coords, obst.radius)){
//                is_inside_an_obstacle = true;
//                break;
//            }
//        }
//        if (is_inside_an_obstacle) {
//            if (Vec3::distance_between_two_vec3(rnd_point, goal_point) < goal_radius) {
//                return find_way_from_goal_to_root(closest->children.back().get());
//            }
//        }
//        else {
//            closest->children.back()->add_child(goal_point);
//            return find_way_from_goal_to_root(closest->children.back()->children.back().get());
//        }

        if (Vec3::distance_between_two_vec3(rnd_point, goal_point) < goal_radius) {
            return find_way_from_goal_to_root(closest->children.back().get());
        }
    }
}
