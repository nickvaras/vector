cmake_minimum_required(VERSION 2.8.3)
project(mecanum_effort)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  vector_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs vector_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/mecanum_effort_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)