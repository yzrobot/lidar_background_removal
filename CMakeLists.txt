cmake_minimum_required(VERSION 2.8.3)
project(lidar_background_removal)

find_package(catkin REQUIRED COMPONENTS roscpp tf sensor_msgs geometry_msgs nav_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/lidar_background_removal.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
