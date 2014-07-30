cmake_minimum_required(VERSION 2.8.3)
project(moveit_humanoid_stability)

find_package(catkin REQUIRED COMPONENTS
  hrl_kinematics
  moveit_core
  moveit_visual_tools
  roscpp
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

# Catkin
catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    roscpp
  INCLUDE_DIRS 
    include
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

# Library
add_library(${PROJECT_NAME}
  src/humanoid_stability.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

# Install
install(TARGETS ${PROJECT_NAME} 
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Mark header files for installation
install(DIRECTORY include/moveit_humanoid_stability
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)