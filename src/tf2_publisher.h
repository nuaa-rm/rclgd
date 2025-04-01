//
// Created by mijiao on 25-3-31.
//

#ifndef RCLGD_TF2_PUBLISHER_H
#define RCLGD_TF2_PUBLISHER_H

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/classes/engine.hpp>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "ros2_node.h"
#include "ros2_communication.h"
#include "util.h"

namespace godot {
    class Tf2Publisher : public Ros2Communication {
    GDCLASS(Tf2Publisher, Ros2Communication)

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

        Node3D *src_node;
        Node3D *dst_node;

        NodePath src_path;
        NodePath dst_path;

        StringName src_name;
        StringName dst_name;

    protected:
        static void _bind_methods() {
            ClassDB::bind_method(D_METHOD("set_src_path", "value"), &Tf2Publisher::set_src_path);
            ClassDB::bind_method(D_METHOD("get_src_path"), &Tf2Publisher::get_src_path, "Node3D");

            ClassDB::add_property("Tf2Publisher",
                                  PropertyInfo(Variant::NODE_PATH, "src_path"),
                                  "set_src_path",
                                  "get_src_path");

            ClassDB::bind_method(D_METHOD("set_dst_path", "value"), &Tf2Publisher::set_dst_path);
            ClassDB::bind_method(D_METHOD("get_dst_path"), &Tf2Publisher::get_dst_path, "Node3D");

            ClassDB::add_property("Tf2Publisher",
                                  PropertyInfo(Variant::NODE_PATH, "dst_path"),
                                  "set_dst_path",
                                  "get_dst_path");

            ClassDB::bind_method(D_METHOD("set_src_name", "value"), &Tf2Publisher::set_src_name);
            ClassDB::bind_method(D_METHOD("get_src_name"), &Tf2Publisher::get_src_name);

            ClassDB::add_property("Tf2Publisher",
                                  PropertyInfo(Variant::STRING_NAME, "src_name"),
                                  "set_src_name",
                                  "get_src_name");

            ClassDB::bind_method(D_METHOD("set_dst_name", "value"), &Tf2Publisher::set_dst_name);
            ClassDB::bind_method(D_METHOD("get_dst_name"), &Tf2Publisher::get_dst_name);

            ClassDB::add_property("Tf2Publisher",
                                  PropertyInfo(Variant::STRING_NAME, "dst_name"),
                                  "set_dst_name",
                                  "get_dst_name");
        };

    public:
        void set_src_path(NodePath value) {
            src_path = value;
        }

        NodePath get_src_path() const {
            return src_path;
        }

        void set_dst_path(NodePath value) {
            dst_path = value;
        }

        NodePath get_dst_path() const {
            return dst_path;
        }

        void set_src_name(StringName value) {
            if (!Engine::get_singleton()->is_editor_hint() && is_node_ready()) {
                return;
            }
            src_name = std::move(value);
            update_configuration_warnings();
        }

        StringName get_src_name() const {
            return src_name;
        }

        void set_dst_name(StringName value) {
            if (!Engine::get_singleton()->is_editor_hint() && is_node_ready()) {
                return;
            }
            dst_name = std::move(value);
            update_configuration_warnings();
        }

        StringName get_dst_name() const {
            return dst_name;
        }

        PackedStringArray _get_configuration_warnings() const override {
            PackedStringArray warnings = Ros2Communication::_get_configuration_warnings();

            if (src_name.is_empty()) {
                warnings.append("Property src_name is empty!");
            }

            if (!src_name.is_valid_ascii_identifier()) {
                warnings.append("Property src_name is invalid!");
            }

            if (dst_name.is_empty()) {
                warnings.append("Property dst_name is empty!");
            }

            if (!dst_name.is_valid_ascii_identifier()) {
                warnings.append("Property dst_name is invalid!");
            }

            return warnings;
        }

        void _enter_tree() override {
            bool is_valid = test_valid();
            if (!is_valid) {
                return;
            }
            tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*node);
        }

        void publisher_data(Transform3D transform) {
            geometry_msgs::msg::TransformStamped transform_stamped;
            Quaternion quaternion = transform.basis.get_quaternion();
            Vector3 translation = transform.origin;
            transform_stamped.transform.translation.set__x(translation.z).set__y(translation.x).set__z(translation.y);
            transform_stamped.transform.rotation.set__x(quaternion.z).set__y(quaternion.x).set__z(quaternion.y).set__w(
                    quaternion.w);
            transform_stamped.header.frame_id = String(src_name).utf8().ptr();
            transform_stamped.child_frame_id = String(dst_name).utf8().ptr();
            transform_stamped.header.stamp = node->now();
            tf_broadcaster->sendTransform(transform_stamped);
        }

        void _process(double delta) override {
            src_node = get_node<Node3D>(src_path);
            dst_node = get_node<Node3D>(dst_path);
            if (!tf_broadcaster || !src_node || !dst_node) {
                return;
            }
            Transform3D from_global = src_node->get_global_transform();
            Transform3D to_global = dst_node->get_global_transform();
            publisher_data(from_global.affine_inverse() * to_global);
        }

    };
}

#endif //RCLGD_TF2_PUBLISHER_H
