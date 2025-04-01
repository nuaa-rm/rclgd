//
// Created by mijiao on 24-11-29.
//

#ifndef RCLGD_IMAGE_PUBLISHER_H
#define RCLGD_IMAGE_PUBLISHER_H

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/sub_viewport.hpp>
#include <godot_cpp/classes/viewport_texture.hpp>
#include <godot_cpp/classes/engine.hpp>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <utility>

#include "ros2_node.h"
#include "ros2_communication.h"

namespace godot {
    class ImagePublisher : public Ros2Communication {
    GDCLASS(ImagePublisher, Ros2Communication)

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher;

        double seconds = 0;

        double delta_time = 0;

        NodePath viewport_path;

        StringName topic_name = "image_raw";

        StringName frame_id = "camera_optical_frame";

    protected:
        static void _bind_methods() {
            ClassDB::bind_method(D_METHOD("set_max_fps", "value"), &ImagePublisher::set_max_fps);
            ClassDB::bind_method(D_METHOD("get_max_fps"), &ImagePublisher::get_max_fps);

            ClassDB::add_property("ImagePublisher",
                                  PropertyInfo(Variant::FLOAT, "fps", PROPERTY_HINT_RANGE, "0,300,0.1,or_greater"),
                                  "set_max_fps",
                                  "get_max_fps");

            ClassDB::bind_method(D_METHOD("set_viewport_path", "value"), &ImagePublisher::set_viewport_path);
            ClassDB::bind_method(D_METHOD("get_viewport_path"), &ImagePublisher::get_viewport_path, "SubViewport");

            ClassDB::add_property("ImagePublisher",
                                  PropertyInfo(Variant::NODE_PATH, "viewport_path"),
                                  "set_viewport_path",
                                  "get_viewport_path");

            ClassDB::bind_method(D_METHOD("set_topic_name", "value"), &ImagePublisher::set_topic_name);
            ClassDB::bind_method(D_METHOD("get_topic_name"), &ImagePublisher::get_topic_name);

            ClassDB::add_property("ImagePublisher",
                                  PropertyInfo(Variant::STRING_NAME, "topic_name"),
                                  "set_topic_name",
                                  "get_topic_name");

            ClassDB::bind_method(D_METHOD("set_frame_id", "value"), &ImagePublisher::set_frame_id);
            ClassDB::bind_method(D_METHOD("get_frame_id"), &ImagePublisher::get_frame_id);

            ClassDB::add_property("ImagePublisher",
                                  PropertyInfo(Variant::STRING_NAME, "frame_id"),
                                  "set_frame_id",
                                  "get_frame_id");
        };

    public:
        void set_max_fps(double value) {
            if (value != 0) {
                delta_time = 1 / value;
            } else {
                delta_time = 0;
            }
        }

        double get_max_fps() const {
            if (delta_time != 0) {
                return 1 / delta_time;
            } else {
                return 0;
            }
        }

        void set_viewport_path(NodePath value) {
            viewport_path = std::move(value);
        }

        NodePath get_viewport_path() const {
            return viewport_path;
        }

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

        void set_frame_id(StringName value) {
            if (!Engine::get_singleton()->is_editor_hint() && is_node_ready()) {
                return;
            }
            frame_id = std::move(value);
            update_configuration_warnings();
        }

        StringName get_frame_id() const {
            return frame_id;
        }

        void _enter_tree() override {
            bool is_valid = test_valid();
            if (!is_valid) {
                return;
            }
            if (!topic_name.is_valid_ascii_identifier()) {
                return;
            }
            image_publisher = node->create_publisher<sensor_msgs::msg::Image>(String(topic_name).utf8().ptr(), 5);
            camera_info_publisher = node->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 5);
        }

        void _process(double delta) override {
            if (Engine::get_singleton()->is_editor_hint()) {
                return;
            }

            if (image_publisher == nullptr || camera_info_publisher == nullptr) {
                return;
            }

            seconds += delta;
            if (delta_time != 0 && seconds < delta_time) {
                return;
            }
            seconds = 0;

            bool use_screen = false;
            if (viewport_path.is_empty()) {
                use_screen = true;
            }
            Viewport *source;
            if (use_screen) {
                source = get_viewport();
            } else {
                auto viewport = get_node<SubViewport>(viewport_path);
                if (viewport == nullptr) {
                    source = get_viewport();
                } else {
                    source = viewport;
                }
            }

            auto img = source->get_texture()->get_image();

            if (img->is_empty()) {
                return;
            }
            double width = img->get_width();
            double height = img->get_height();

            auto image_msg = std::make_unique<sensor_msgs::msg::Image>();

            image_msg->header.stamp = node->now();
            image_msg->header.frame_id = String(frame_id).utf8().ptr();

            image_msg->height = (int32_t) height;
            image_msg->width = (int32_t) width;

            image_msg->encoding = "rgb8";
            image_msg->is_bigendian = false;
            image_msg->step = img->get_data().size() / image_msg->height;
            image_msg->data.resize(img->get_data().size());
            std::memcpy(&image_msg->data[0], img->get_data().ptrw(), img->get_data().size());
            image_publisher->publish(std::move(image_msg));

            auto camera_info = std::make_unique<sensor_msgs::msg::CameraInfo>();

            camera_info->height = (int32_t) height;
            camera_info->width = (int32_t) width;
            camera_info->distortion_model = "plumb_bob";
            camera_info->d = {0.0, 0.0, 0.0, 0.0, 0.0};

            Camera3D *camera = source->get_camera_3d();
            double theta = camera->get_fov() / 180.0 * M_PI;

            double phi = 2.0 * atan(24.0 * tan(theta / 2) / 36.0);

            double fx = width / (2.0 * tan(theta / 2.0));
            double fy = height / (2.0 * tan(phi / 2.0));


            camera_info->k = {fx, 0.0, width / 2.0,
                              0.0, fy, height / 2.0,
                              0.0, 0.0, 1.0};
            camera_info_publisher->publish(std::move(camera_info));
        }

    };
}


#endif //RCLGD_IMAGE_PUBLISHER_H
