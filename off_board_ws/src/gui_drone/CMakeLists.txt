cmake_minimum_required(VERSION 3.16.3)
project(gui_drone)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Quick REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)


set(THIS_PACKAGE_INCLUDE_DEPENDS
    ament_index_cpp
    rclcpp
    std_msgs
)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

add_executable(gui_node 
  src/gui_node.cpp
  src/custom_widgets/lineedit.cpp
  include/gui_drone/custom_widgets/lineedit.hpp
  # include/gui_drone/*
)

target_include_directories(gui_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(gui_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(gui_node ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(gui_node
  ${moveit_ros_planning_LIBRARIES}
  Qt5::Core
  Qt5::Quick
  Qt5::Widgets
)
install(TARGETS gui_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
    resources
  DESTINATION share/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
