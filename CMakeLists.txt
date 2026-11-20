# Main CMake file
cmake_minimum_required(VERSION 3.30)

# Project name, version and languages
project(Salvage_Belt VERSION 0.5.1 LANGUAGES C CXX)

# Configure build options
set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# The project uses headers. GCC module-scanning flags are incompatible with clang-tidy.
set(CMAKE_CXX_SCAN_FOR_MODULES OFF CACHE BOOL "Scan C++ sources for module dependencies.")

# Configuration to set binary output and modules directory.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

# Set Sanitizer preset options
set(SANITIZER_PRESET "default" CACHE STRING "Default sanitizer preset used by sanitizer helpers.")
set_property(CACHE SANITIZER_PRESET PROPERTY STRINGS default debug thread memory leak)

# Options
option(SALVAGE_ENABLE_THREADS "Enable the persistent CPU worker pool." ON)
option(BUILD_TESTS "Build Catch2 unit tests." OFF)
option(BUILD_TESTS_WITH_MAIN "Build Catch2 Tests with custom main function." OFF)
option(ENABLE_WARNINGS "Enable compiler warnings." OFF)
option(ENABLE_WARNINGS_AS_ERRORS "Enable compiler warnings as errors." OFF)
option(ENABLE_CLANG_TIDY "Enable clang-tidy static analysis." OFF)
option(ENABLE_SANITIZERS "Enable sanitizer helpers for targets." OFF)
option(ENABLE_LTO_GLOBALY "Enable LTO/IPO globally for Release and RelWithDebInfo targets." ON)

# Modules include
include(LTO)
if(PROJECT_IS_TOP_LEVEL)
    include(Docs)
endif()
include(Warnings)
include(ClangTidy)
include(Sanitizer)
if(BUILD_TESTS)
    include(CTest)
    enable_testing()
endif()

# Subdirectories
add_subdirectory(src)
add_subdirectory(includes)
if(PROJECT_IS_TOP_LEVEL)
    add_subdirectory(config)
    add_subdirectory(app)
endif()
add_subdirectory(tests)
