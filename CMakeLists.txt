cmake_minimum_required(VERSION 3.16)

project(OrbitCore VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(orbitcore
    src/Physics.cpp
    src/SpaceBase.cpp
    src/Spacecraft.cpp
    src/Vector3.cpp
)

target_include_directories(orbitcore
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)

if(MSVC)
    target_compile_options(orbitcore PRIVATE /W4 /permissive-)
else()
    target_compile_options(orbitcore PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(orbitcore_sim src/main.cpp)
target_link_libraries(orbitcore_sim PRIVATE orbitcore)

enable_testing()

add_executable(orbitcore_tests tests/OrbitCoreTests.cpp)
target_link_libraries(orbitcore_tests PRIVATE orbitcore)
add_test(NAME orbitcore_tests COMMAND orbitcore_tests)
