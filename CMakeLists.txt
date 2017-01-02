cmake_minimum_required(VERSION 2.8.3)
project(arl_hw)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/Modules/")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -O0")
set(DCMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
        control_toolbox
        controller_manager
        diagnostic_updater
        hardware_interface
        arl_controllers
        arl_interfaces
        arl_hw_msgs
        realtime_tools
        roscpp
        )

find_package(WiringPi REQUIRED)
find_package(BCM2835 REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)


catkin_package(
        INCLUDE_DIRS include
        LIBRARIES arl_hw
        CATKIN_DEPENDS control_toolbox controller_manager hardware_interface roscpp
#        DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${WIRINGPI_INCLUDE_DIRS}
        ${BCM2835_INCLUDE_DIRS}
        )

## Declare a C++ library
add_library(arl_robot
        src/robot.cpp
        src/raspberry_pi.cpp
        src/raspberry_pi_spi.cpp
        src/dummy.cpp
        src/ad5360.cpp
        )

add_executable(arl_driver_node src/driver.cpp src/driver_utils.cpp src/rt_history.cpp)

target_link_libraries(arl_robot
        ${catkin_LIBRARIES}
        ${WIRINGPI_LIBRARIES}
        ${BCM2835_LIBRARIES}
        )

target_link_libraries(arl_driver_node
        rt
        arl_robot
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        )
