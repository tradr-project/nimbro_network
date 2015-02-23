cmake_minimum_required(VERSION 2.8)

project(nimbro_topic_transport)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	topic_tools
	rostest
)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

execute_process(
	COMMAND ${CMAKE_CXX_COMPILER} -dumpversion
	OUTPUT_VARIABLE GCC_VERSION
)

if(NOT (GCC_VERSION VERSION_GREATER 4.7 OR GCC_VERSION VERSION_EQUAL 4.7))
	message(WARNING "C++11 not supported, compiler version ${GCC_VERSION}")
	return()
endif()

find_library(BZ2_LIBRARY bz2 REQUIRED)


set(CMAKE_CXX_FLAGS "-Wall -std=c++11")
add_executable(udp_sender
	src/udp/topic_sender.cpp
	src/udp/bandwidth_control.cpp
	src/udp/udp_sender.cpp
)

target_link_libraries(udp_sender
	${catkin_LIBRARIES}
	${BZ2_LIBRARY}
)

add_executable(udp_receiver
	src/udp/udp_receiver.cpp
	src/topic_info.cpp
)

target_link_libraries(udp_receiver
	${catkin_LIBRARIES}
	${BZ2_LIBRARY}
)

add_executable(tcp_sender
	src/tcp/tcp_sender.cpp
	src/topic_info.cpp
)
target_link_libraries(tcp_sender
	${catkin_LIBRARIES}
	${BZ2_LIBRARY}
)

add_executable(tcp_receiver
	src/tcp/tcp_receiver.cpp
	src/topic_info.cpp
)
target_link_libraries(tcp_receiver
	${catkin_LIBRARIES}
	${BZ2_LIBRARY}
)

# Tests
add_rostest_gtest(test_comm test/topic_transport.test
	test/test_comm.cpp
)
target_link_libraries(test_comm
	${catkin_LIBRARIES}
)