cmake_minimum_required(VERSION 3.0.2)
project(bachelor)

# if(DEFINED ENV{VIRTUAL_ENV} OR DEFINED ENV{CONDA_PREFIX})
#   set(_pip_args)
# else()
#   set(_pip_args "--user")
# endif()

# set(_pypkg "pandas tdqm")

# execute_process(COMMAND pip install ${_pip_args} ${_pypkg})
execute_process(
  COMMAND python3 -m pip install pandas tdqm seaborn gitpython 
  COMMAND python3 -m pip install -U Pillow requests setuptools matplotlib
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  cv_bridge
  std_msgs
  image_transport
  pcl_conversions
  pcl_ros
  ackermann_msgs
  nav_msgs
  tf
)
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES bachelor
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)
include_directories(
  include/bachelor
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(
  example_node
  src/example.cpp
  )

add_dependencies(example_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_node
  ${catkin_LIBRARIES}
)

add_executable(
  transform_pc2_node
  src/transform_pc2.cpp
  )
add_dependencies(transform_pc2_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(transform_pc2_node
    ${catkin_LIBRARIES}
  )

install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
  PATTERN ".svn" EXCLUDE)
