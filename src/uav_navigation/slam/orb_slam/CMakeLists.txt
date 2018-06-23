cmake_minimum_required(VERSION 2.8.3)
project(orb_slam)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_msgs
  geometry_msgs
  mavros_msgs
)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES simple_line_planner
        #  CATKIN_DEPENDS other_catkin_pkg
        #  DEPENDS system_lib
)

include_directories(include
${catkin_INCLUDE_DIRS}
)

add_executable(test_fix_point test/test_fix_point.cpp)
target_link_libraries(test_fix_point ${catkin_LIBRARIES})

add_executable(test_mavros_vel_control test/test_mavros_vel_control.cpp)
target_link_libraries(test_fix_point ${catkin_LIBRARIES})


install(TARGETS test_fix_point test_mavros_vel_control
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
