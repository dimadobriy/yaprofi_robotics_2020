cmake_minimum_required(VERSION 2.8.3)
project(bac_task)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    message_generation
    tf
    visp_bridge
)

find_package(VISP REQUIRED)

add_message_files(
  FILES
  CartesianTrajectory.msg
  CameraFeatures.msg
)

add_service_files(
  FILES
  SetTrajectoryParameters.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES bac_task
  CATKIN_DEPENDS roscpp rospy sensor_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
     include
    ${catkin_INCLUDE_DIRS}
    ${VISP_INCLUDE_DIRS}
)

add_executable(marker_detector_node src/marker_detector_node.cpp)
target_link_libraries(marker_detector_node ${catkin_LIBRARIES} ${VISP_LIBRARIES})
add_dependencies(marker_detector_node ${catkin_EXPORTED_TARGETS})

#add_executable(main_solve_node src/main_solve.cpp)
#target_link_libraries(main_solve_node ${catkin_LIBRARIES} ${VISP_LIBRARIES})
#add_dependencies(main_solve_node ${catkin_EXPORTED_TARGETS})
