cmake_minimum_required(VERSION 2.8.3)
project(cooperation_server)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(cmake_modules REQUIRED COMPONENTS eigen octomap)
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs octomap_ros pcl_ros octomap_msgs)
find_package(octomap REQUIRED)

catkin_package(
        INCLUDE_DIRS ${OCTOMAP_INCLUDE_DIRS}
        LIBRARIES ${PROJECT_NAME} ${OCTOMAP_LIBRARIES}
        DEPENDS sensor_msgs tf octomap_msgs pcl_ros octomap_ros
        DEPENDS eigen
)

include_directories(
    ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/src
)

add_executable(cooperationServer src/cooperationServer.cpp src/SLAMserver.cpp)
target_link_libraries(cooperationServer ${catkin_LIBRARIES})
