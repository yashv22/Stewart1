cmake_minimum_required(VERSION 3.0.2)
project(stewart_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gazebo_msgs
  geometry_msgs
)

# Find Eigen library
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs gazebo_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}  # Include Eigen headers
)

# Add the executable
add_executable(code src/code.cpp)  # Adjust the file name if needed
target_link_libraries(code ${catkin_LIBRARIES})

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/stewart_command_publisher.py
  scripts/stewart_control.py
  scripts/shaft_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Ensure the Python scripts have execute permission
file(GLOB scripts "${PROJECT_SOURCE_DIR}/scripts/*.py")
foreach(script ${scripts})
  execute_process(COMMAND chmod +x ${script})
endforeach()

