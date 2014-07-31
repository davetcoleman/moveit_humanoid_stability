cmake_minimum_required(VERSION 2.8.3)
project(moveit_humanoid_stability)

find_package(catkin REQUIRED COMPONENTS
  hrl_kinematics
  moveit_core
  moveit_visual_tools
  pluginlib
  roscpp
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

# Catkin
catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    hrl_kinematics
    moveit_core
    moveit_visual_tools
  INCLUDE_DIRS 
    include
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

# Stability Library
add_library(${PROJECT_NAME}
  src/humanoid_stability.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES}
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

# Sampling Library
add_library(humanoid_moveit_constraint_sampler 
  src/humanoid_constraint_sampler.cpp
)
target_link_libraries(humanoid_moveit_constraint_sampler 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
add_dependencies(humanoid_moveit_constraint_sampler ${catkin_EXPORTED_TARGETS})

# Install library
install(
  TARGETS 
    ${PROJECT_NAME} 
    humanoid_moveit_constraint_sampler
  LIBRARY DESTINATION 
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install header files
install(
  DIRECTORY 
    include
  DESTINATION 
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install plugins
install(
  FILES
    humanoid_constraint_sampler_plugin.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

