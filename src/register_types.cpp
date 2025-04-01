#include <gdextension_interface.h>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>
#include <godot_cpp/classes/engine.hpp>

#include "register_types.h"
#include "image_publisher.h"
#include "tf2_publisher.h"
#include "ros2_publisher.h"
#include "ros2_subscription.h"

#include "rclcpp/rclcpp.hpp"

using namespace godot;

void initialize_module(ModuleInitializationLevel p_level) {
    if (p_level == MODULE_INITIALIZATION_LEVEL_CORE){
        try{
            rclcpp::init(0, nullptr);
        }catch(const std::exception& e){
            print_error(e.what());
        }
    }

    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
        return;
    }

    GDREGISTER_CLASS(Ros2Node)
    GDREGISTER_CLASS(Ros2Communication)
    GDREGISTER_CLASS(Tf2Publisher)
    GDREGISTER_CLASS(ImagePublisher)
    GDREGISTER_CLASS(Ros2Publisher)
    GDREGISTER_CLASS(Ros2Subscription)
}

void terminate_module(ModuleInitializationLevel p_level) {
    if (p_level == MODULE_INITIALIZATION_LEVEL_CORE){
        try{
            rclcpp::shutdown();
        }catch(const std::exception& e){
            print_error(e.what());
        }
    }
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
        return;
    }
}

extern "C" {
// Initialization.

GDExtensionBool GDE_EXPORT
rclgd_library_init(GDExtensionInterfaceGetProcAddress
                   p_get_proc_address, const GDExtensionClassLibraryPtr p_library,
                   GDExtensionInitialization *r_initialization) {
    godot::GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);

    init_obj.register_initializer(initialize_module);
    init_obj.register_terminator(terminate_module);
    init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);

    return init_obj.init();
}
}