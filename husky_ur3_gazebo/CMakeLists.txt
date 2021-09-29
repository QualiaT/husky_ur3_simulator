cmake_minimum_required(VERSION 2.8.3)
project(husky_ur3_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslaunch
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 노드 추가 - 210927 YSW
add_executable(gazebo_rh_pub src/gazebo_rh_pub.cpp)
add_dependencies(gazebo_rh_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_rh_pub ${catkin_LIBRARIES})

install(TARGETS gazebo_rh_pub
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch controller DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
