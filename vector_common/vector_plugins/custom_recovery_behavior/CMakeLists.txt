cmake_minimum_required(VERSION 2.8.3)
project(custom_recovery)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  tf
  costmap_2d
  nav_core
  pluginlib
  base_local_planner
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES custom_recovery
  CATKIN_DEPENDS
    roscpp
    tf
    costmap_2d
    nav_core
    pluginlib
    base_local_planner
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(custom_recovery src/${PROJECT_NAME}.cpp)

target_link_libraries(custom_recovery ${catkin_LIBRARIES})

install(
  FILES custom_recovery_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
