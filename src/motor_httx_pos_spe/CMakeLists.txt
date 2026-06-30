cmake_minimum_required(VERSION 3.8)
project(motor_httx_pos_spe)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(base_interfaces_demo REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(tiaozi_tcp_model_fixed_cars
  src/tiaozi_tcp_model_fixed_cars.cpp
)
target_compile_features(tiaozi_tcp_model_fixed_cars PUBLIC cxx_std_17)
target_link_libraries(tiaozi_tcp_model_fixed_cars Eigen3::Eigen)
ament_target_dependencies(tiaozi_tcp_model_fixed_cars
  rclcpp
  ament_index_cpp
  base_interfaces_demo
  geometry_msgs
)

install(TARGETS
  tiaozi_tcp_model_fixed_cars
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
