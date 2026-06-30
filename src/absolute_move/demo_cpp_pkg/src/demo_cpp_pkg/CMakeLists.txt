cmake_minimum_required(VERSION 3.8)
project(demo_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(base_interfaces_demo REQUIRED)

find_package(Qt5 COMPONENTS Widgets Concurrent REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

include_directories(
  include
)

add_executable(tiaozi src/tiaozi.cpp)
ament_target_dependencies(tiaozi
  rclcpp
  geometry_msgs
  base_interfaces_demo
  Eigen3
)
target_link_libraries(tiaozi Eigen3::Eigen)

add_executable(tiaozi_gui 
  src/tiaozi_gui.cpp
  src/main.cpp
  include/demo_cpp_pkg/tiaozi_gui.h
)
ament_target_dependencies(tiaozi_gui
  rclcpp
  geometry_msgs
  base_interfaces_demo
  Eigen3
)
target_link_libraries(tiaozi_gui 
  Eigen3::Eigen
  Qt5::Widgets
  Qt5::Concurrent
)

install(TARGETS
  tiaozi
  tiaozi_gui
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  ui
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()