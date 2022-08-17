# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(map_listener)


## Find catkin and any catkin packages
find_package(catkin REQUIRED
    COMPONENTS
        roscpp
        rospy
        std_msgs
        sensor_msgs
        nav_msgs
        message_generation
)

## Declare ROS messages and services
add_message_files(
    FILES
        PoseVel.msg
        OccupancyGridUpdate.msg
        PointCloud2Update.msg
        ProjectedMapInfo.msg
        ProjectedMap.msg
)

add_service_files(
    FILES
        GetMapROI.srv
        GetPointMapROI.srv
        GetPointMap.srv
        ProjectedMapsInfo.srv
        SaveMap.srv
        SetMapProjections.srv
)

## Generate added messages and services
generate_messages(
    DEPENDENCIES
        std_msgs
        sensor_msgs
        nav_msgs
)

## Declare a catkin package
catkin_package(
    CATKIN_DEPENDS
        message_runtime
        std_msgs
        sensor_msgs
        nav_msgs
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS scripts/mapListener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Build listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(mapListener src/mapListener.cpp)
target_link_libraries(mapListener ${catkin_LIBRARIES})
add_dependencies(mapListener map_listener_generate_messages_cpp)

# %EndTag(FULLTEXT)%
