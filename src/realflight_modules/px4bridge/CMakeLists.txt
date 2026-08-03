cmake_minimum_required(VERSION 3.0.2)
project(px4bridge)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(px4bridge_node
  src/px4bridge_node.cc
)

target_compile_features(px4bridge_node PRIVATE cxx_std_14)
target_compile_options(px4bridge_node PRIVATE
  -Wall
  -Wextra
  -Wpedantic
)

target_link_libraries(px4bridge_node
  ${catkin_LIBRARIES}
)

install(TARGETS px4bridge_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md LICENSE
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
