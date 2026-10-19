cmake_minimum_required(VERSION 3.21)
project(armeo_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies


add_subdirectory(xpctarget_api_cpp)

add_executable(teleop
    src/teleop.cpp
)
target_link_libraries(teleop xpcapi)

target_include_directories(teleop PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(teleop PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(test_traj
    src/test_traj.cpp
)
target_link_libraries(test_traj xpcapi)

target_include_directories(test_traj PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(record_replay
    src/record_replay.cpp
)
target_link_libraries(record_replay xpcapi)

target_include_directories(record_replay PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(print_state
    src/print_state.cpp
)
target_link_libraries(print_state xpcapi)

target_include_directories(print_state PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(vel_control
    src/vel_control.cpp
)
target_link_libraries(vel_control xpcapi)

target_include_directories(vel_control PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(collect_data
    src/collect_data.cpp
)
target_link_libraries(collect_data xpcapi)

target_include_directories(collect_data PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(test_dh
    src/test_dh.cpp
)
target_link_libraries(test_dh xpcapi)

target_include_directories(test_dh PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# target_compile_options(teleop PRIVATE -fsanitize=address -g -O0)
# target_link_options(teleop PRIVATE -fsanitize=address)