cmake_minimum_required(VERSION 3.8)
project(drone_description)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    urdf
    worlds
    models
    config
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES
    lib/libPriusHybridPlugin.so
  DESTINATION lib
)

ament_package()