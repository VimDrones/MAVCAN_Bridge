# Library directory CMakeLists.txt
cmake_minimum_required(VERSION 3.22)

# Check for MAVLink
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/mavlink/common/mavlink.h")
    message(STATUS "MAVLink library not found. Consider running:")
    message(STATUS "  git submodule update --init --recursive")
else()
    # Create MAVLink library target
    add_library(mavlink INTERFACE)
    
    # Configure include paths
    target_include_directories(mavlink INTERFACE 
        ${CMAKE_CURRENT_SOURCE_DIR}/mavlink
        ${CMAKE_CURRENT_SOURCE_DIR}/mavlink/common
    )
    
    # Set compile definitions
    target_compile_definitions(mavlink INTERFACE 
        MAVLINK_COMM_NUM_BUFFERS=2
    )
    
    message(STATUS "MAVLink library configured")
endif()