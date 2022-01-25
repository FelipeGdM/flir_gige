cmake_minimum_required(VERSION 3.5)
project(flir_gige)

add_definitions(-std=c++14 -Wall -Wno-deprecated-declarations -Wextra -Wno-unknown-pragmas)

# Set library paths for pleora ebus sdk on 64-bit linux
# For more information, see cmake/Findebus.cmake
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    # Make CMake aware of the cmake folder for local FindXXX scripts.
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
    set(Ebus_HINTS /opt/pleora/ebus_sdk/Ubuntu-14.04-x86_64)
    find_package(Ebus REQUIRED)
else()
    message(FATAL_ERROR "This package only supports 64-bit linux")
endif()

# find_package(catkin REQUIRED COMPONENTS
#     roscpp nodelet camera_base cv_bridge
#     )
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(builtin_interfaces REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)

find_package(OpenCV)

# Dynamic reconfigure option, as to be placed right before catkin_package
# generate_dynamic_reconfigure_options(
#     cfg/FlirGigeDyn.cfg
#     cfg/ThermalProcDyn.cfg
#     cfg/CalibProcDyn.cfg
#     )

include_directories(
    include
    ${Ebus_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

# library
add_library(${PROJECT_NAME}
    src/flir_gige/flir_gige.cpp
    src/flir_gige/flir_gige_ros.cpp
    src/flir_gige/flir_gige_node.cpp
    # src/flir_gige/flir_gige_nodelet.cpp
    # src/thermal_proc/thermal_proc_node.cpp
    # src/thermal_proc/thermal_proc_nodelet.cpp
    # src/calib_proc/calib_proc_node.cpp
    )

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBRARIES}
    ${Ebus_LIBRARIES}
    )

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    sensor_msgs
    diagnostic_msgs
    camera_info_manager
    image_transport
)

# node
add_executable(${PROJECT_NAME}_node
    src/flir_gige/flir_gige_main.cpp
    )
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    )

# add_executable(thermal_proc_node
#     src/thermal_proc/thermal_proc_main.cpp
#     )
# target_link_libraries(thermal_proc_node
#     ${PROJECT_NAME}
#     )
# add_executable(calib_proc_node
#     src/calib_proc/calib_proc_main.cpp
#     )
# target_link_libraries(calib_proc_node
#     ${PROJECT_NAME}
#     )

install(TARGETS ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(camera_info_manager
    diagnostic_updater
    image_transport
    builtin_interfaces
)

# Instead of catkin_package
ament_package()

## Add cmake target dependencies of the executable/library
# add_dependencies(${PROJECT_NAME}
    # ${catkin_EXPORTED_TARGETS}
    # ${PROJECT_NAME}_gencfg
    # ${PROJECT_NAME}_gencpp
    # ${${PROJECT_NAME}_EXPORTED_TARGETS}
    # )
