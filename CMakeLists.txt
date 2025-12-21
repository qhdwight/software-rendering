cmake_minimum_required(VERSION 4.1)
project(Engine)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -fno-exceptions -fno-rtti -ffp-contract=fast")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion -Wimplicit-fallthrough")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-missing-designated-field-initializers")
set(CMAKE_CXX_STANDARD 26)

add_executable(engine main.cpp)
target_include_directories(engine PUBLIC ${CMAKE_CURRENT_LIST_DIR})
