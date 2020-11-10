cmake_minimum_required(VERSION 2.8.3)
project(rviz_tool_cursor)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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

# Mark executables and/or libraries for installation
install(TARGETS tool_cursor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

