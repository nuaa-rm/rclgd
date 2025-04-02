//
// Created by mijiao on 25-3-31.
//

#ifndef RCLGD_ROS2_PUBLISHER_H
#define RCLGD_ROS2_PUBLISHER_H

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/classes/engine.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>

#include <rosidl_runtime_c/string.h>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/transform_stamped.h>

#include "ros2_communication.h"
#include "util.h"

namespace godot {
    class Ros2Publisher : public Ros2Communication {
    GDCLASS(Ros2Publisher, Ros2Communication)

        StringName topic_name;
        StringName message_type;
        rclcpp::GenericPublisher::SharedPtr publisher;

        Dictionary message_info;

    protected:
        static void _bind_methods() {
            ClassDB::bind_method(D_METHOD("set_topic_name", "value"), &Ros2Publisher::set_topic_name);
            ClassDB::bind_method(D_METHOD("get_topic_name"), &Ros2Publisher::get_topic_name);

            ClassDB::add_property("Ros2Publisher",
                                  PropertyInfo(Variant::STRING_NAME, "topic_name"),
                                  "set_topic_name",
                                  "get_topic_name");

            ClassDB::bind_method(D_METHOD("set_message_type", "value"), &Ros2Publisher::set_message_type);
            ClassDB::bind_method(D_METHOD("get_message_type"), &Ros2Publisher::get_message_type);

            ClassDB::add_property("Ros2Publisher",
                                  PropertyInfo(Variant::STRING_NAME, "message_type"),
                                  "set_message_type",
                                  "get_message_type");

            ClassDB::bind_method(D_METHOD("publish", "message"), &Ros2Publisher::publish);
        };

    public:
        void set_topic_name(StringName value) {
            if (!Engine::get_singleton()->is_editor_hint() && is_node_ready()) {
                return;
            }
            topic_name = std::move(value);
            update_configuration_warnings();
        }

        StringName get_topic_name() const {
            return topic_name;
        }

        void set_message_type(StringName value) {
            if (!Engine::get_singleton()->is_editor_hint() && is_node_ready()) {
                return;
            }
            message_type = std::move(value);
            update_configuration_warnings();
        }

        StringName get_message_type() const {
            return message_type;
        }

        bool test_valid() override {
            bool ret = Ros2Communication::test_valid();
//            if (!topic_name.is_valid_ascii_identifier()) {
//                return false;
//            }
            return ret;
        }

        void _enter_tree() override {
            bool is_valid = test_valid();
            if (!is_valid) {
                return;
            }
            publisher = node->create_generic_publisher(
                    String(topic_name).utf8().ptr(),
                    String(message_type).utf8().ptr(),
                    10
            );
            message_info = get_message_structure(message_type);
        }

        void publish(const Dictionary &dictionary) {
            auto msg = std::make_shared<rclcpp::SerializedMessage>();
            int64_t offset = 4;
            PackedByteArray array = {0, 1, 0, 0};
            msg_to_bin(dictionary, message_info, offset, array);
            msg->reserve(array.size());
            memcpy(msg->get_rcl_serialized_message().buffer, array.ptr(), array.size());
            msg->get_rcl_serialized_message().buffer_length = array.size();
            msg->get_rcl_serialized_message().buffer_capacity = array.size();
            publisher->publish(*msg);
        }

        void _process(double delta) override {
            bool is_valid = test_valid();
            if (!is_valid) {
                return;
            }
        }
    };
}

#endif //RCLGD_ROS2_PUBLISHER_H
