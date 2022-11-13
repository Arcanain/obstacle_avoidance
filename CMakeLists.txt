cmake_minimum_required(VERSION 3.0.2)
project(obstacle_avoidance)

find_package(catkin REQUIRED
    roscpp
    sensor_msgs
    geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES obstacle_avoidance
   CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(obstacle_avoidance src/obstacle_avoidance.cpp)
target_link_libraries(obstacle_avoidance ${catkin_LIBRARIES})

add_executable(obstacle_avoidance_rplidar src/obstacle_avoidance_rplidar.cpp)
target_link_libraries(obstacle_avoidance_rplidar ${catkin_LIBRARIES})

add_executable(obstacle_avoidance_rplidar_test src/obstacle_avoidance_rplidar_test.cpp)
target_link_libraries(obstacle_avoidance_rplidar_test ${catkin_LIBRARIES})