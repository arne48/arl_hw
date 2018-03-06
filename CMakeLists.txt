cmake_minimum_required(VERSION 3.1)
project(arl_hw)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/Modules/")

#
#IMPORTANT!!! Due to the defines used in the bcm2835 library
#             the driver can just be build with either Raspberry Pi
#             OR Tinker Board support
#
OPTION(BUILD_RPI "Build with Raspberry Pi 3 support" ON)
OPTION(BUILD_LINUX_PLATFORM "Build with Linux spidev and sysfs support" OFF)
OPTION(BUILD_TINKERBOARD "Build with Asus Tinkerboard support" OFF)

##Uncomment for Debug Build
#set(DCMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")

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
        std_srvs
        )

find_package (Threads REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES arl_hw
        CATKIN_DEPENDS control_toolbox controller_manager hardware_interface roscpp
#       DEPENDS system_lib
)

SET(SOURCES
        src/robot.cpp
        src/dummy.cpp
        src/ad5360.cpp
        src/ad7616.cpp
        src/ad7730.cpp
        )

SET(ROBOT_LIBRARIES
        ${catkin_LIBRARIES}
        )

SET(INCLUDE_DIRS
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        )

IF(BUILD_RPI)
        MESSAGE(STATUS "Using platform code for the Raspberry Pi 3")
        add_definitions(-DFOR_RPI)
        find_package(BCM2835 REQUIRED)

        LIST(APPEND SOURCES
        src/raspberry_pi.cpp
        src/raspberry_pi_spi.cpp
        src/raspberry_pi_gpio.cpp
                )

        LIST(APPEND ROBOT_LIBRARIES
        ${BCM2835_LIBRARIES}
                )

        LIST(APPEND INCLUDE_DIRS
        ${BCM2835_INCLUDE_DIRS}
                )
ENDIF()

IF(BUILD_TINKERBOARD)
        MESSAGE(STATUS "Using platform code for the Asus Tinkerboard")
        add_definitions(-DFOR_TINKERBOARD)
        find_package(TINKERBOARD_IO REQUIRED)

        LIST(APPEND SOURCES
        src/tinkerboard.cpp
        src/tinkerboard_spi.cpp
        src/tinkerboard_gpio.cpp
                )

        LIST(APPEND ROBOT_LIBRARIES
                ${TINKERBOARD_IO_LIBRARIES}
                )

        LIST(APPEND INCLUDE_DIRS
                ${TINKERBOARD_IO_INCLUDE_DIRS}
                )
ENDIF()

IF(BUILD_LINUX_PLATFORM)
        MESSAGE(STATUS "Using platform code for the Linux spidev and sysfs")
        add_definitions(-DFOR_LINUX_PLATFORM)
        LIST(APPEND SOURCES
        src/linux_platform.cpp
        src/linux_platform_spi.cpp
        src/linux_platform_gpio.cpp
                )
ENDIF()

include_directories(
        ${INCLUDE_DIRS}
)

## Declare a C++ library
add_library(arl_robot ${SOURCES})

target_link_libraries(arl_robot
        ${ROBOT_LIBRARIES}
        )

add_executable(arl_driver_node src/driver.cpp src/driver_utils.cpp src/rt_history.cpp)

target_link_libraries(arl_driver_node
        rt
        arl_robot
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        )

##test files

#add_executable(test_rpi_bcm2835 tests/rpi_bcm2835_test.cpp)
#
#target_link_libraries(test_rpi_bcm2835
#        ${catkin_LIBRARIES}
#        ${BCM2835_LIBRARIES}
#        )
