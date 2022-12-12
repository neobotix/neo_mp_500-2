cmake_minimum_required(VERSION 3.5)

project(neo_mp_500-2)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(neo_srvs2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(configure_relays src/configure_relays.cpp)

ament_target_dependencies(configure_relays
  rclcpp
  neo_srvs2
  geometry_msgs
  nav_msgs
  )

target_compile_features(configure_relays PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS configure_relays
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch configs robot_model
    
  DESTINATION share/${PROJECT_NAME})

ament_package()
