cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set_property(GLOBAL PROPERTY CXX_STANDARD 11)

find_package(gazebo REQUIRED)
find_library(EXO_LIB NAMES libexo exo)

include_directories(${GAZEBO_INCLUDE_DIRS}, ../../)
link_directories(${GAZEBO_LIBRARY_DIRS})
execute_process(COMMAND exo magicdef .. OUTPUT_VARIABLE MAGIC_FLAGS)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} ${MAGIC_FLAGS}")

add_library(collector SHARED collector.cpp)
target_link_libraries(collector ${GAZEBO_LIBRARIES} ${EXO_LIB})