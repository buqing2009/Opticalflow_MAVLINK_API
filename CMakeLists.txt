cmake_minimum_required(VERSION 2.8)
PROJECT(optiflow_mav_api C CXX)  

FIND_PACKAGE ( Threads REQUIRED )

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/include/mavlink )
LINK_DIRECTORIES( ${PROJECT_SOURCE_DIR}/lib)

#SET(SOURCE_FILES main.cpp)

ADD_LIBRARY(optiflow_mav_api STATIC 
	    src/autopilot_interface.cpp src/serial_port.cpp)
	    
	    
ADD_EXECUTABLE(optiflow_mav src/mavlink_control.cpp)

TARGET_LINK_LIBRARIES( optiflow_mav optiflow_mav_api ${CMAKE_THREAD_LIBS_INIT} )
