cmake_minimum_required(VERSION 3.0.2)
project(imu_integrator)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES Imu_Integrator
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(imu_integrator_node src/imu_integrator_node.cpp
                                   src/imu_integrator.cpp
                                   )
target_link_libraries(imu_integrator_node ${catkin_LIBRARIES})