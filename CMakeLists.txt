cmake_minimum_required(VERSION 2.8.17)


project(kobuki)


add_compile_options(-Ofast -fno-math-errno -finline-functions -Wall -Wextra -Wpedantic -std=c++17 -std=gnu++17) #-Werror)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(ALL_LIBRARIES)


find_package(Threads REQUIRED)


add_library(kobuki STATIC
    kobuki/kobuki.cpp
    kobuki/kobuki.h
    kobuki/log.cpp
    kobuki/log.h
    kobuki/messages.h
    kobuki/protocol.h
)
target_include_directories(kobuki PRIVATE kobuki)
target_link_libraries(kobuki -pthread)


macro(app progname source)
    add_executable(${progname} ${source})
    target_include_directories(${progname} PRIVATE kobuki)
    target_link_libraries(${progname} -pthread -lncurses kobuki)
endmacro()

app(teleop apps/teleop.cpp)
app(jingle apps/jingle.cpp)
