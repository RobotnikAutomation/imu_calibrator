cmake_minimum_required(VERSION 2.8.3)
project(imu_calibrator)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
)

add_message_files(DIRECTORY msg
    FILES
    CalibratorStatus.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS
    message_runtime
    std_msgs
)