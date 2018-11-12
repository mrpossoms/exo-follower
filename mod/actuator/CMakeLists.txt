cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
find_library(EXO_LIB NAMES libexo exo)

include_directories(${GAZEBO_INCLUDE_DIRS}, ../../)
link_directories(${GAZEBO_LIBRARY_DIRS})
execute_process(COMMAND exo magicdef .. OUTPUT_VARIABLE MAGIC_FLAGS)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} ${MAGIC_FLAGS}")
set (CMAKE_CXX_STANDARD 11)

add_library(actuator SHARED actuator.cpp)
target_link_libraries(actuator ${GAZEBO_LIBRARIES} ${EXO_LIB})
