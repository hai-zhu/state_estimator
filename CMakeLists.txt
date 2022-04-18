cmake_minimum_required(VERSION 2.8.3)
project(state_estimator)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        nav_msgs
        tf
)

find_package(Eigen3 REQUIRED)

catkin_package(
        CATKIN_DEPENDS 
        roscpp rospy 
        std_msgs 
        geometry_msgs
        nav_msgs 
        tf 
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        include/${PROJECT_NAME}
)

add_executable(mocap_state_kalman_filter_node src/mocap_state_kf_node.cpp src/mocap_state_kf.cpp src/kalman_filter.cpp)
target_link_libraries(mocap_state_kalman_filter_node ${catkin_LIBRARIES})

# set(CMAKE_BUILD_TYPE Release)

## For debugging
# set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}  -g")
# set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_VERBOSE_MAKEFILE ON)
