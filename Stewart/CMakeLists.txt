cmake_minimum_required(VERSION 3.0.2)
project(stewart)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  
)

# Find Eigen3
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(ik src/ik.cpp)
target_link_libraries(ik
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

# Install Python scripts
catkin_install_python(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Ensure the Python scripts have execute permission
file(GLOB scripts "${PROJECT_SOURCE_DIR}/scripts/*.py")
foreach(script ${scripts})
  execute_process(COMMAND chmod +x ${script})
endforeach()

