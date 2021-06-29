cmake_minimum_required(VERSION 3.0.2)
project(tracer_base)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
## Set compiler to use c++ 17 features
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# cmake module path
#list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS    
#    roslaunch
#    roslint
    roscpp 
    sensor_msgs
    std_msgs
    geometry_msgs
    tracer_msgs
   #tracer_sdk
    tf2 
    tf2_ros
    ugv_sdk
)
    
# find_package(Boost REQUIRED COMPONENTS chrono)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tracer_messenger
# CATKIN_DEPENDS tracer_msgs roscpp sensor_msgs
#   DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(tracer_messenger STATIC src/tracer_messenger.cpp)
target_link_libraries(tracer_messenger ${catkin_LIBRARIES})
set_property(TARGET tracer_messenger PROPERTY POSITION_INDEPENDENT_CODE ON)

add_executable(tracer_base_node src/tracer_base_node.cpp)
target_link_libraries(tracer_base_node tracer_messenger ${catkin_LIBRARIES})

add_executable(tracer_base_sim_node src/tracer_base_sim_node.cpp)
target_link_libraries(tracer_base_sim_node tracer_messenger ${catkin_LIBRARIES})

#############
## Install ##
#############

#roslaunch_add_file_check(launch)

#install(TARGETS tracer_messenger tracer_base_node
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#install(DIRECTORY launch urdf
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
