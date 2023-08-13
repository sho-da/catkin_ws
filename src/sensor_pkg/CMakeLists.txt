cmake_minimum_required(VERSION 3.0.2)
project(sensor_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)
catkin_package()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pthread")

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(bmx055_kalman_publisher src/bmx055_kalman_publisher.cpp)
target_link_libraries(bmx055_kalman_publisher
  ${catkin_LIBRARIES}
  pigpiod_if2
  pthread
  rt
)
install(TARGETS bmx055_kalman_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(encoder_publisher src/encoder_publisher.cpp)
target_link_libraries(encoder_publisher
  ${catkin_LIBRARIES}
  pigpiod_if2
  pthread
  rt
)
install(TARGETS encoder_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(driver_subscriber src/driver_subscriber.cpp)
target_link_libraries(driver_subscriber
  ${catkin_LIBRARIES}
  pigpiod_if2
  pthread
  rt
)
install(TARGETS driver_subscriber
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
