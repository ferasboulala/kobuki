cmake_minimum_required(VERSION 2.8.11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Werror -Wall -Wextra -Wformat=0 -Werror=vla -DNDEBUG -O2 -finline-functions")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/../bin/")

project(kobuki)
add_executable(kobuki_test test.cpp)
target_link_libraries(kobuki_test)
