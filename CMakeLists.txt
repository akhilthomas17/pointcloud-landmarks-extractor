cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pole_detector)

find_package(PCL 1.3 REQUIRED COMPONENTS common io) # Change here, add components as needed

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
	${PCL_COMMON_LIBRARIES} 
	${PCL_IO_LIBRARIES}) # Change here, link component libraries as needed
