//
// Created by misha on 10/17/21.
//

#pragma once

#include "Vec3.h"
#include <memory>
#include <vector>
#include <iostream>
#include <queue>
#include <algorithm>
#include "Node.h"
#include "Object.h"
#include "World.h"

class RRT_tree{
public:
    std::shared_ptr<Node> root;
    World *world_p;
    
    explicit RRT_tree(Vec3 &coords, World *ptr_to_world);
    
    [[nodiscard]] static std::vector<Vec3> find_way_from_goal_to_root(Node* last_goal_p) ; // TODO what is it
    
    static std::vector<Vec3> find_path_to_goal(World *world_ptr, Vec3& start_point, Vec3& goal_point);
};

