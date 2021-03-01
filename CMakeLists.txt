cmake_minimum_required(VERSION 2.8.3)
project(rviz_tool_cursor)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

cmake_policy(SET CMP0057 NEW)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(pluginlib REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io geometry)
#find_package(pcl_ros REQUIRED)

find_package(Eigen3 REQUIRED)

# QT
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)

qt5_wrap_cpp(MOC_FILES
  include/${PROJECT_NAME}/rviz_tool_cursor.h
  src/circle_tool_cursor.h
  src/mesh_tool_cursor.h
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/rviz_tool_cursor.cpp
  src/${PROJECT_NAME}/get_point_on_plane.cpp
  src/circle_tool_cursor.cpp
  src/mesh_tool_cursor.cpp
  ${MOC_FILES}
)
target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
  ${rclcpp_LIBRARIES}
  ${rviz_common_LIBRARIES}
  ${rviz_rendering_LIBRARIES}
  ${pluginlib_LIBRARIES}
  Eigen3::Eigen
  ${PCL_LIBRARIES})
target_include_directories(${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PUBLIC $<INSTALL_INTERFACE:include>)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${PCL_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC
    ${pluginlib_INCLUDE_DIRS}
    ${rviz_common_INCLUDE_DIRS}
    ${rviz_rendering_INCLUDE_DIRS}

    )
target_compile_definitions(${PROJECT_NAME} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION include
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY resources
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${PROJECT_NAME}
    rclcpp
    rviz_common
    rviz_rendering
    pluginlib)

ament_package()
