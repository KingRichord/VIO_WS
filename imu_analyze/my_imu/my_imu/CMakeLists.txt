cmake_minimum_required(VERSION 3.0.2)
project(my_imu)
# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    tf
    geometry_msgs
)
set(serial_SRCS
        serial/src/serial.cc
        serial/include/serial/serial.h
        serial/include/serial/v8stdint.h
        )
include_directories(serial/include)
# If unix
list(APPEND serial_SRCS serial/src/impl/unix.cc)
list(APPEND serial_SRCS serial/src/impl/list_ports/list_ports_linux.cc)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_imu
#  CATKIN_DEPENDS roscpp sensor_msg
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME}_node
        src/main.cpp  src/zyf.cpp ${serial_SRCS}
        )

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        )