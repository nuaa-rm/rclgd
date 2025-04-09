//
// Created by mijiao on 25-3-31.
//

#ifndef RCLGD_ROS2_SUBSCRIPTION_H
#define RCLGD_ROS2_SUBSCRIPTION_H

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/classes/engine.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>

#include "ros2_communication.h"
#include "util.h"

namespace godot {
    class Ros2Subscription : public Ros2Communication {
    GDCLASS(Ros2Subscription, Ros2Communication)

        StringName topic_name = "test";
        StringName message_type = "geometry_msgs/msg/TransformStamped";
        rclcpp::GenericSubscription::SharedPtr subscription;

        Dictionary message_info;

    protected:
        static void _bind_methods() {
            ClassDB::bind_method(D_METHOD("set_topic_name", "value"), &Ros2Subscription::set_topic_name);
            ClassDB::bind_method(D_METHOD("get_topic_name"), &Ros2Subscription::get_topic_name);

            ClassDB::add_property("Ros2Subscription",
                                  PropertyInfo(Variant::STRING_NAME, "topic_name"),
                                  "set_topic_name",
                                  "get_topic_name");

            ClassDB::bind_method(D_METHOD("set_message_type", "value"), &Ros2Subscription::set_message_type);
            ClassDB::bind_method(D_METHOD("get_message_type"), &Ros2Subscription::get_message_type);

            ClassDB::add_property("Ros2Subscription",
                                  PropertyInfo(Variant::STRING_NAME, "message_type"),
                                  "set_message_type",
                                  "get_message_type");

            ClassDB::add_signal("Ros2Subscription",
                                MethodInfo("message_received", PropertyInfo(Variant::DICTIONARY, "message")));
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

        PackedStringArray _get_configuration_warnings() const override {
            PackedStringArray warnings = Ros2Communication::_get_configuration_warnings();

            if (topic_name.is_empty()) {
                warnings.append("Property topic_name is empty!");
            }

//            if (!topic_name.is_valid_ascii_identifier()) {
//                warnings.append("Property topic_name is invalid!");
//            }

            return warnings;
        }

        void _enter_tree() override {
            bool is_valid = test_valid();
            if (!is_valid) {
                return;
            }
            try {
                subscription = node->create_generic_subscription(
                        String(topic_name).utf8().ptr(),
                        String(message_type).utf8().ptr(),
                        rclcpp::SensorDataQoS(),
                        std::bind(&Ros2Subscription::callback, this, std::placeholders::_1)
                );
            } catch (const std::exception &e) {
                print_error(e.what());
                return;
            }

            message_info = get_message_structure(message_type);
        }

        void callback(const std::shared_ptr<const rclcpp::SerializedMessage> &serializedMessage) {
            PackedByteArray array;
            array.resize(serializedMessage->get_rcl_serialized_message().buffer_capacity);
            memcpy(array.ptrw(), serializedMessage->get_rcl_serialized_message().buffer,
                   serializedMessage->get_rcl_serialized_message().buffer_length);
            int64_t offset = 4;
            Dictionary msg = bin_to_msg(array, message_info, offset);
            emit_signal("message_received", msg);
        }
    };
}


#endif //RCLGD_ROS2_SUBSCRIPTION_H
