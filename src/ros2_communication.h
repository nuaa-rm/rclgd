//
// Created by mijiao on 25-3-31.
//

#ifndef RCLGD_ROS2_COMMUNICATION_H
#define RCLGD_ROS2_COMMUNICATION_H

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/classes/engine.hpp>

#include <rclcpp/rclcpp.hpp>

#include <utility>

#include "ros2_node.h"

namespace godot {
    class Ros2Communication : public Node {
    GDCLASS(Ros2Communication, Node)

    protected:
        rclcpp::Node::SharedPtr node;
        bool enable = true;

        static void _bind_methods() {
            ClassDB::bind_method(D_METHOD("set_enable", "value"), &Ros2Communication::set_enable);
            ClassDB::bind_method(D_METHOD("get_enable"), &Ros2Communication::get_enable);
            ClassDB::add_property("Ros2Communication",
                                  PropertyInfo(Variant::BOOL, "enable"),
                                  "set_enable",
                                  "get_enable");
        };

    public:
        void set_enable(bool value) {
            enable = value;
        }

        bool get_enable() const {
            return enable;
        }

        PackedStringArray _get_configuration_warnings() const override {
            PackedStringArray warnings;

            Node *parent = get_parent();
            if (parent == nullptr) {
                warnings.append("The parent of this node should be a child node!");
            } else {
                auto *ros_parent = Object::cast_to<Ros2Node>(parent);
                if (ros_parent == nullptr) {
                    warnings.append("The parent of this node should be Ros2Node!");
                }
            }

            return warnings;
        }

        virtual bool test_valid(){
            if (Engine::get_singleton()->is_editor_hint()) {
                return false;
            }
            auto *ros_2_node = Object::cast_to<Ros2Node>(get_parent());
            if (ros_2_node == nullptr) {
                print_error("The parent of this node should be Ros2Node!");
                return false;
            }
            node = ros_2_node->get_node();
            if (!node) {
                print_error("node is empty! Please check node status!");
                return false;
            }
            return true;
        }
    };
}

#endif //RCLGD_ROS2_COMMUNICATION_H
