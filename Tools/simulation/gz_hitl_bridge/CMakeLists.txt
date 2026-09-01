cmake_minimum_required(VERSION 3.16)
project(gz_hitl_bridge CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Wno-unused-parameter -Wno-address-of-packed-member)

# --- gz-transport / gz-msgs (Harmonic) ---
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 QUIET)
if(NOT gz-msgs10_FOUND)
    find_package(gz-msgs11 REQUIRED)
    set(GZ_MSGS_TARGET gz-msgs11::gz-msgs11)
else()
    set(GZ_MSGS_TARGET gz-msgs10::gz-msgs10)
endif()
find_package(gz-math7 REQUIRED)

# --- yaml-cpp for multi-vehicle config files ---
find_package(yaml-cpp REQUIRED)

# --- MAVLink C headers (reuse PX4's generated headers) ---
if(NOT DEFINED MAVLINK_INCLUDE_DIR)
    file(GLOB _mavlink_candidates
        "${CMAKE_CURRENT_SOURCE_DIR}/../../../build/*/mavlink"
    )
    list(LENGTH _mavlink_candidates _n)
    if(_n EQUAL 0)
        message(FATAL_ERROR
            "Could not find MAVLink headers. Build PX4 once (e.g. `make px4_sitl_default`) "
            "or pass -DMAVLINK_INCLUDE_DIR=/path/to/mavlink/include")
    endif()
    list(GET _mavlink_candidates 0 MAVLINK_INCLUDE_DIR)
endif()
message(STATUS "Using MAVLink headers: ${MAVLINK_INCLUDE_DIR}")

add_executable(gz_hitl_bridge
    src/main.cpp
    src/transport_serial.cpp
    src/transport_udp.cpp
    src/transport_factory.cpp
    src/actuator.cpp
    src/preset.cpp
    src/vehicle.cpp
    src/vehicle_manager.cpp
    src/config.cpp
)

target_include_directories(gz_hitl_bridge PRIVATE
    ${MAVLINK_INCLUDE_DIR}
    ${MAVLINK_INCLUDE_DIR}/common
    src
)

target_link_libraries(gz_hitl_bridge PRIVATE
    gz-transport13::gz-transport13
    ${GZ_MSGS_TARGET}
    gz-math7::gz-math7
    yaml-cpp
    pthread
)
