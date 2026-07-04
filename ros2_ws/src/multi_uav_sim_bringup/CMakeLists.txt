cmake_minimum_required(VERSION 3.8)
project(multi_uav_sim_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

