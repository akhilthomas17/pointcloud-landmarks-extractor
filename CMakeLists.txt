cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pole_detector)

find_package(PCL 1.3 REQUIRED)

include_directories(
${PCL_INCLUDE_DIRS}
include
)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(pole_detector
	src/pole_detector.cpp
	src/main.cpp)

target_link_libraries(pole_detector 
	${PCL_LIBRARIES})
