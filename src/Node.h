//
// Created by misha on 10/12/21.
//

#pragma once

#include "Vec3.h"
#include <memory>
#include <vector>
#include <iostream>
#include <queue>


class Node {
public:
    Vec3 coords;
    Node *parent;
    std::vector<std::shared_ptr<Node>> children;
    
    Node(Vec3 &coords, Node *parent);
    
    explicit Node(Vec3 &coords);
    
    void add_child(Vec3 &coords);
    
    void print_out_all_children();
    
    static std::shared_ptr<Node> find_the_closest_node(Vec3 &point, Node *root_node);
};
