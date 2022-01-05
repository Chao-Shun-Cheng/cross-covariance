cmake_minimum_required(VERSION 3.0.2)
project(cross_covariance)

find_package(autoware_build_flags REQUIRED)
find_package(autoware_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  pcl_ros
  geometry_msgs
  tf
  vector_map
  lanelet2_extension
  autoware_msgs
  amathutils_lib
  )

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS
  roscpp
  pcl_ros
  autoware_msgs
  tf
  vector_map
  amathutils_lib
  )

set(ROSLINT_CPP_OPTS "--filter=-build/c++14")

# cross_covariance
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cross_cov
        src/main.cpp
        src/cross_covariance.cpp
        )
target_link_libraries(cross_cov
        ${catkin_LIBRARIES}
        )
add_dependencies(cross_cov
        ${catkin_EXPORTED_TARGETS}
        )

install(TARGETS
        cross_cov
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)

if (CATKIN_ENABLE_TESTING)
  roslint_add_test()
endif()
