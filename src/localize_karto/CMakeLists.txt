cmake_minimum_required(VERSION 3.1)
project(slam_karto)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    message_filters
    nav_msgs
#    open_karto
    rosconsole
    roscpp
    sensor_msgs
    std_srvs
    tf 
)

#find_package(Boost REQUIRED COMPONENTS thread)
#find_package(Eigen3 REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
#    rosbag
    roscpp
    sensor_msgs
    tf
  DEPENDS Boost
  INCLUDE_DIRS include
  LIBRARIES correlation_scan_match
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(correlation_scan_match
                                   src/correlation_scan_match.cpp

)
target_link_libraries(correlation_scan_match ${Boost_LIBRARIES})
install( TARGETS correlation_scan_match
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


add_executable(slam_karto src/slam_karto.cpp)
target_link_libraries(slam_karto correlation_scan_match ${catkin_LIBRARIES} ${Boost_LIBRARIES})


install(TARGETS slam_karto
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

