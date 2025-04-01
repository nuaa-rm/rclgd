//
// Created by mijiao on 25-3-31.
//

#ifndef RCLGD_ROS2_NODE_H
#define RCLGD_ROS2_NODE_H

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/classes/engine.hpp>

#include <rclcpp/rclcpp.hpp>
#include <utility>

#include "util.h"

namespace godot {
    class Ros2Node : public Node {
    GDCLASS(Ros2Node, Node)

        rclcpp::Node::SharedPtr node;
        StringName node_name;

    protected:
        static void _bind_methods() {
            ClassDB::bind_method(D_METHOD("set_node_name", "value"), &Ros2Node::set_node_name);
            ClassDB::bind_method(D_METHOD("get_node_name"), &Ros2Node::get_node_name);

            ClassDB::add_property("Ros2Node",
                                  PropertyInfo(Variant::STRING_NAME, "node_name"),
                                  "set_node_name",
                                  "get_node_name");
        };

    public:
        void set_node_name(StringName value) {
            if (!Engine::get_singleton()->is_editor_hint() && is_node_ready()) {
                return;
            }
            node_name = std::move(value);
            update_configuration_warnings();
        }

        StringName get_node_name() const {
            return node_name;
        }

        rclcpp::Node::SharedPtr get_node() {
            return node;
        }

        PackedStringArray _get_configuration_warnings() const override {
            PackedStringArray warnings;

            if (node_name.is_empty()) {
                warnings.append("Property node_name is empty!");
            }

            if (!node_name.is_valid_ascii_identifier()) {
                warnings.append("Property node_name is invalid!");
            }

            return warnings;
        }

        void _enter_tree() override {
            if (Engine::get_singleton()->is_editor_hint()) {
                return;
            }
            std::string s = String(node_name).utf8().ptr();
            if (!s.empty() && node_name.is_valid_ascii_identifier()) {
                node = std::make_shared<rclcpp::Node>(s);
            }
        }

        void _process(double delta) override {
            if (Engine::get_singleton()->is_editor_hint()) {
                return;
            }
            if (node) {
                rclcpp::spin_some(node);
            }
        }
    };
}

#endif //RCLGD_ROS2_NODE_H
