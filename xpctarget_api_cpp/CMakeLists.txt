
cmake_minimum_required(VERSION 3.15...3.30)
project(
    XPCAPICPP
    VERSION 1.0.0
    DESCRIPTION "A modern C++ project template"
    LANGUAGES CXX
)

# Force out-of-source builds to keep the source tree clean
if(CMAKE_BINARY_DIR STREQUAL CMAKE_SOURCE_DIR)
    message(FATAL_ERROR "In-source builds are forbidden. Please create a build/ directory.")
endif()

set(CMAKE_CXX_STANDARD 17)                  # Options: 11, 14, 17, 20, 23
set(CMAKE_CXX_STANDARD_REQUIRED ON)         # Fallback to older standards disabled
set(CMAKE_CXX_EXTENSIONS OFF)               # Disables compiler-specific extensions (e.g., gnu++17)

# Set default build type to Release if none is provided
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose build type." FORCE)
endif()


# find_package(Threads REQUIRED)
# find_package(fmt CONFIG REQUIRED) # Assumes dependency via vcpkg or conan


# Collect source files explicitly (Avoid wildcard/GLOB as it breaks build triggers)
add_library(xpcapi SHARED
    src/xpcapi.cpp
    src/xpc_msg.cpp
    src/xpc_comms.cpp
)

# Include directories mapped explicitly to this target
target_include_directories(xpcapi
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE 
        src
)

# target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(xpctest
    src/test.cpp
)
target_link_libraries(xpctest xpcapi)

