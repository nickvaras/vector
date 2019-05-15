cmake_minimum_required(VERSION 2.8.3)
project(laserscan_to_pcl2)

## Uncomment if the package has a setup.py
#catkin_python_setup()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp nodelet laser_geometry )

## Declare a catkin package
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES laserscan_to_pcl2
    CATKIN_DEPENDS nodelet roscpp laser_geometry
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(laserscan_to_pcl2 src/laserscan_to_pcl2_nodelet.cpp)
target_link_libraries(laserscan_to_pcl2 ${catkin_LIBRARIES})

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(laserscan_to_pcl2 ${catkin_EXPORTED_LIBRARIES})
endif()

install(TARGETS laserscan_to_pcl2
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# install(DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



