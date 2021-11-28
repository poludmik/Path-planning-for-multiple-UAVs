//
// Created by misha on 10/12/21.
//

#pragma once

#include "Vec3.h"
#include <memory>
#include <vector>
#include <iostream>
#include <queue>
#include <algorithm>

class Node {
public:
    Vec3 coords;
    Node *parent = nullptr;
    bool inside_the_goal = false;
    std::vector<std::shared_ptr<Node>> children;
    double cost;
    
    Node(const Vec3 &coords, Node *parent);
    
    explicit Node(const Vec3 &coords);
    
    void set_as_final();
    
    void add_child(const Vec3 &coords);

    void change_parent(Node *new_parent);
    
    void print_out_all_children() const;
    
    static Node* find_the_closest_node(const Vec3 &point, Node *root_node);

    static std::vector<Node *> get_neighbors_in_radius(Node *root,
                                                       const Vec3& point,
                                                       double radius);
};
