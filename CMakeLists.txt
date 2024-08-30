cmake_minimum_required(VERSION 3.0.2)
project(rviz_obstacle_generator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  hmcl_msgs
  dynamic_reconfigure
  eigen_conversions
)

find_package(Eigen3 REQUIRED)


# Generate dynamic reconfigure options
generate_dynamic_reconfigure_options(
  cfg/Obstacle.cfg
  cfg/Obstaclev2.cfg
)

catkin_package(
  CATKIN_DEPENDS roscpp visualization_msgs dynamic_reconfigure
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rviz_obstacle_generator_node src/rviz_obstacle_generator.cpp)
add_dependencies(rviz_obstacle_generator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(rviz_obstacle_generator_node  ${catkin_LIBRARIES} )

add_executable(rviz_obstacle_generator_node_v2 src/rviz_obstacle_generator_v2.cpp)
add_dependencies(rviz_obstacle_generator_node_v2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(rviz_obstacle_generator_node_v2  ${catkin_LIBRARIES} )
