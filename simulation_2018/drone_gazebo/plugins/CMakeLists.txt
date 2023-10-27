find_package(roscpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${roscpp_INCLUDE_DIRS})

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# Create the gimbal_control_plugin library.
add_library(${PROJECT_NAME}_gimbal_control_plugin gimbal_control_plugin.cc gimbal_control_plugin.hh)
target_link_libraries(${PROJECT_NAME}_gimbal_control_plugin ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
