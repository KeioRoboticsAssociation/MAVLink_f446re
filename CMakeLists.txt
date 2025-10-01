cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 20)

# Suppress common MAVLink warnings for cleaner builds
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter -Wno-address-of-packed-member -Wno-pedantic")


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME mavlink_test)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Create motor core library for shared interfaces
add_library(motor_core INTERFACE)
target_compile_features(motor_core INTERFACE cxx_std_20)
target_compile_definitions(motor_core INTERFACE
    USE_COMPILE_TIME_CONFIG
    MOTOR_COUNT=6
    $<$<CONFIG:Debug>:DEBUG>
    $<$<CONFIG:Debug>:ENABLE_DEBUG_LOGGING>
    $<$<CONFIG:Debug>:ENABLE_ASSERTIONS>
)

# Add new architecture sources
file(GLOB_RECURSE APP_SOURCES App/*.cpp App/*.c)
file(GLOB_RECURSE USER_SOURCES Src/*.c Src/*.cpp)
file(GLOB_RECURSE USER_LIB_SOURCES Lib/*/Src/*.c Lib/*/Src/*.cpp)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # New architecture sources
    ${APP_SOURCES}
    # Legacy sources (will be gradually removed)
    ${USER_SOURCES}
    ${USER_LIB_SOURCES}
)

# Add include paths for new architecture
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # New architecture includes
    App
    App/config
    App/hal
    App/motors
    App/comm
    App/storage
    App/safety
    # Legacy includes
    Inc
    Lib
)

# Add legacy library include directories
file(GLOB USER_LIB_DIRS Lib/*/Inc)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${USER_LIB_DIRS}
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    motor_core
    # Add user defined libraries
)
