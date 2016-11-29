cmake_minimum_required(VERSION 2.8.3)
project(common_interfaces)

### ROS Dependencies ###
# Find the RTT-ROS package (this transitively includes the Orocos CMake macros)
find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  shm_comm
  )

include_directories(${catkin_INCLUDE_DIRS} include)

if(CATKIN_ENABLE_TESTING)
    find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-transport-corba)
    include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

    catkin_add_gtest(msg_concate_unittest tests/gtest_main.cc tests/msg_concate_unittest.cpp tests/test_ports.cpp tests/msg_split_unittest.cpp)
    target_link_libraries(msg_concate_unittest
        ${catkin_LIBRARIES}
        ${USE_OROCOS_LIBRARIES})

endif(CATKIN_ENABLE_TESTING)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS rtt_ros shm_comm
)


