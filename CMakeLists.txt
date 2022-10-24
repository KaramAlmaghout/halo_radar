cmake_minimum_required(VERSION 2.8.3)
project(halo_radar)

add_compile_options(-std=c++11)


set(OpenCV_DIR "/usr/share/OpenCV/")


find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    marine_sensor_msgs
    tf2_msgs
    geometry_msgs
    OpenCV
    autoware_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 
   CATKIN_DEPENDS marine_sensor_msgs autoware_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_DIR}
)

set (SOURCES
    src/halo_radar/halo_radar.cpp
    src/halo_radar/halo_radar_node.cpp
)


add_executable(${PROJECT_NAME} ${SOURCES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

add_executable(halo_rviz src/halo_rviz.cpp)
add_dependencies(halo_rviz ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(halo_rviz ${catkin_LIBRARIES})
add_executable(radar_img src/radar_img.cpp)
add_dependencies(radar_img ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(radar_img ${OpenCV_LIBRARIES}  ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS scripts/ctrl_radar.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
