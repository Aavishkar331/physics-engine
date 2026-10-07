cmake_minimum_required(VERSION 3.16)
project(physics_engine CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Warnings for GCC/Clang.
add_compile_options(-O2 -Wall -Wextra)

# --- Raylib ---------------------------------------------------------------
# Prefer a system install; otherwise fetch and build it.
find_package(raylib 4.0 QUIET)
if(NOT raylib_FOUND)
    message(STATUS "raylib not found locally; fetching via FetchContent")
    include(FetchContent)
    FetchContent_Declare(
        raylib
        GIT_REPOSITORY https://github.com/raysan5/raylib.git
        GIT_TAG 5.5
    )
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(raylib)
endif()

# --- Engine library -------------------------------------------------------
add_library(physics
    src/Body.cpp
    src/Collision.cpp
    src/World.cpp
)
target_include_directories(physics PUBLIC include)
target_link_libraries(physics PUBLIC raylib)

# --- Application ----------------------------------------------------------
add_executable(engine app/main.cpp)
target_link_libraries(engine PRIVATE physics)
