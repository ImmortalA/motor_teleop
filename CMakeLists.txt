cmake_minimum_required(VERSION 3.10)
project(spine_board)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Threads REQUIRED)
find_path(ASIO_INCLUDE_DIR asio.hpp)

add_library(spine_board SHARED spine_board.cpp)
target_include_directories(spine_board PRIVATE ${ASIO_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(spine_board PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(spine_board Threads::Threads)

add_executable(test_spine apps/main.cpp)
target_link_libraries(test_spine spine_board)
target_include_directories(test_spine PRIVATE ${ASIO_INCLUDE_DIR})
