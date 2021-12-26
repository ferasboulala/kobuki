cmake_minimum_required(VERSION 2.8.17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Werror -Wall -Wextra -Wformat=0 -Werror=vla -DNDEBUG -O2 -finline-functions")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(ALL_LIBRARIES)

project(kobuki)
include_directories("${PROJECT_SOURCE_DIR}/kobuki")
file(GLOB PROJECT_SOURCES "${PROJECT_SOURCE_DIR}/kobuki/*.cpp")
find_package(Threads REQUIRED)

add_executable(teleop
    "${PROJECT_SOURCE_DIR}/apps/teleop.cpp"
    ${PROJECT_SOURCES}
)
target_link_libraries(teleop -pthread -lncurses)

add_executable(jingle
    "${PROJECT_SOURCE_DIR}/apps/jingle.cpp"
    ${PROJECT_SOURCES}
)
set_target_properties(jingle PROPERTIES CXX_STANDARD 17)
target_link_libraries(jingle -pthread)
