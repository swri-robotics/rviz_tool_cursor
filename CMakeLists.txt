cmake_minimum_required(VERSION 2.8.3)
project(rviz_tool_cursor)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rviz
  pluginlib
  pcl_ros
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    tool_cursor
  CATKIN_DEPENDS
    rviz
    pluginlib
    pcl_ros
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# QT
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)

qt5_wrap_cpp(MOC_FILES
  include/${PROJECT_NAME}/rviz_tool_cursor.h
  src/circle_tool_cursor.h
  src/mesh_tool_cursor.h
)

add_library(tool_cursor
  src/${PROJECT_NAME}/rviz_tool_cursor.cpp
  src/circle_tool_cursor.cpp
  src/mesh_tool_cursor.cpp
  ${MOC_FILES}
)
target_link_libraries(tool_cursor
  ${catkin_LIBRARIES}
  Qt5::Widgets
)

#############
## Install ##
#############

#############
## Testing ##
#############

