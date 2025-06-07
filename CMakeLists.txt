cmake_minimum_required(VERSION 3.10)
project(robot LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_COMPILER arm-linux-gnueabihf-g++)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_FIND_ROOT_PATH /usr/arm-linux-gnueabihf)
set(CMAKE_LIBRARY_PATH /usr/arm-linux-gnueabihf/lib /usr/local/lib)

# Find Mosquitto
find_package(PkgConfig REQUIRED)
pkg_check_modules(MOSQUITTO REQUIRED libmosquitto)
if(NOT MOSQUITTO_FOUND)
    message(FATAL_ERROR "Mosquitto library not found")
else()
     message(STATUS "Found MQTT: ${MOSQUITTO_INCLUDE_DIRS}")
endif()
include_directories(${MOSQUITTO_INCLUDE_DIRS})


# Add executable
add_executable(robot robot.cpp)

# Link libraries
target_link_libraries(robot ${MOSQUITTO_LIBRARIES})
target_link_libraries(robot ${MOSQUITTO_LIBRARIES})