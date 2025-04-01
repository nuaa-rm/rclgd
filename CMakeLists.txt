cmake_minimum_required(VERSION 3.8)
project(rclgd)

add_subdirectory(godot-cpp)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY)

set(GODOTCPP_DISABLE_EXCEPTIONS OFF)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

file(GLOB SOURCE RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "src/*.cpp")

ament_auto_add_library(${PROJECT_NAME} SHARED ${SOURCE})

target_link_libraries(${PROJECT_NAME} godot-cpp)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

string(TOLOWER "${CMAKE_SYSTEM_NAME}" SYSTEM_NAME)
string(TOLOWER "${CMAKE_BUILD_TYPE}" BUILD_TYPE)
set(OUTPUT_NAME "${PROJECT_NAME}.${SYSTEM_NAME}.${BUILD_TYPE}.${CMAKE_SYSTEM_PROCESSOR}")

set_target_properties(${PROJECT_NAME}
        PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/${PROJECT_NAME}/bin"
        OUTPUT_NAME "${OUTPUT_NAME}"
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_auto_package()
