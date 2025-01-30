cmake_minimum_required(VERSION 3.16)
project(cgn_ros)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    sensor_msgs
    geometry_msgs
    message_generation
)

add_message_files(
    FILES
    Grasps.msg
)

add_service_files(
    FILES
    GetGrasps.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS
    message_runtime
    std_msgs
    sensor_msgs
    geometry_msgs
)
