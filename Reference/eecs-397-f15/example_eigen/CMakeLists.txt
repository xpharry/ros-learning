cmake_minimum_required(VERSION 2.8.3)
project(example_eigen)

find_package(catkin_simple REQUIRED)
#next line supposedly needed for Eigen, per: https://github.com/ros/cmake_modules/blob/0.3-devel/README.md#usage
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_library(my_lib src/my_lib.cpp)   

# Executables
cs_add_executable(example_eigen_plane_fit src/example_eigen_plane_fit.cpp)
target_link_libraries(example_eigen_plane_fit ${catkin_LIBRARIES})

cs_add_executable(example_eigen_xforms src/example_eigen_xforms.cpp)
target_link_libraries(example_eigen_xforms ${catkin_LIBRARIES})

cs_install()
cs_export()
    
