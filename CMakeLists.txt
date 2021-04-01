cmake_minimum_required(VERSION 2.8.11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++2a -Werror -Wall -Wextra -Wformat=0 -Werror=vla -DNDEBUG -O2 -finline-functions")
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(ALL_LIBRARIES)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/../bin/")

project(kobuki)
include_directories("${PROJECT_SOURCE_DIR}/kobuki")
file(GLOB PROJECT_SOURCES "${PROJECT_SOURCE_DIR}/kobuki/*.cpp")

add_executable(kobuki_test
    "${PROJECT_SOURCE_DIR}/test/test.cpp"
    ${PROJECT_SOURCES}
)
target_link_libraries(kobuki_test)
