cmake_minimum_required(VERSION 2.8.3)
project(joints_controller)
find_package(catkin_simple REQUIRED)

catkin_simple()

# Executables
cs_add_executable(joint1_controller src/joint1_controller.cpp)
cs_add_executable(joint1_commander src/joint1_commander.cpp)
cs_add_executable(joint2_controller src/joint2_controller.cpp)
cs_add_executable(joint2_commander src/joint2_commander.cpp)

cs_install()
cs_export()
