cmake_minimum_required(VERSION 3.26)
cmake_policy(SET CMP0048 NEW)

# Fetch libDaisy
include(FetchContent)
FetchContent_Declare(daisy SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/../../LIBS/DAISY/libDaisy)
FetchContent_MakeAvailable(daisy)

FetchContent_Declare(daisysp SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/../../LIBS/DAISY/DaisySP)
FetchContent_MakeAvailable(daisysp)

# Add optimization flags for size
add_compile_options(
  -Os # Optimize for size
  -flto # Enable link-time optimization
  -ffunction-sections # Place each function in its own section
  -fdata-sections # Place each data item in its own section
)

add_link_options(
  -flto
  -Wl,--gc-sections # Remove unused sections
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Our project declaration
project(vollkorn VERSION 0.0.1)

set(FIRMWARE_NAME ${CMAKE_PROJECT_NAME})
set(FIRMWARE_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/src/vollkorn.cpp
)

# DaisyProject.cmake uses FIRMWARE_NAME and FIRMWARE_SOURCES to build a target called ${FIRMWARE_NAME}
include(DaisyProject)

set_target_properties(${FIRMWARE_NAME} PROPERTIES
  CXX_STANDARD 23
  CXX_STANDARD_REQUIRED YES
)
target_link_libraries(${FIRMWARE_NAME} PUBLIC DaisySP)
