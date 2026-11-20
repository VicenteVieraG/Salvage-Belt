if(NOT TARGET CLI11::CLI11)
    add_subdirectory("${PROJECT_SOURCE_DIR}/external/cli11" "${PROJECT_BINARY_DIR}/external/cli11" EXCLUDE_FROM_ALL)
endif()

add_executable(salvage-headless main.cpp Options.cpp Benchmark.cpp Report.cpp)
target_link_libraries(salvage-headless PRIVATE salvage::core CLI11::CLI11)
target_set_warnings(salvage-headless ${ENABLE_WARNINGS_AS_ERRORS})
target_enable_clang_tidy(salvage-headless)
target_enable_sanitizers(salvage-headless)

# Only the export code needs these headers. A core-only build needs neither submodule.
target_include_directories(salvage-headless SYSTEM PRIVATE
    "${PROJECT_SOURCE_DIR}/external/json/include"
    "${PROJECT_SOURCE_DIR}/external/csv/include")
target_compile_definitions(salvage-headless PRIVATE CSV_ENABLE_THREADS=0)

find_package(Git QUIET)
if(Git_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
    execute_process(COMMAND "${GIT_EXECUTABLE}" describe --always --dirty --abbrev=12
        WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}" OUTPUT_VARIABLE revision
        OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
endif()
if(NOT revision)
    set(revision "unknown")
endif()
set(build_flags "${CMAKE_CXX_FLAGS}")
if(CMAKE_CONFIGURATION_TYPES)
    set(configurations ${CMAKE_CONFIGURATION_TYPES})
elseif(CMAKE_BUILD_TYPE)
    set(configurations ${CMAKE_BUILD_TYPE})
endif()
foreach(config IN LISTS configurations)
    string(TOUPPER "${config}" upper)
    string(APPEND build_flags " $<$<CONFIG:${config}>:${CMAKE_CXX_FLAGS_${upper}}>")
endforeach()
configure_file(BuildInfo.hpp.in BuildInfo.gen.hpp @ONLY)
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/BuildInfo.hpp"
    INPUT "${CMAKE_CURRENT_BINARY_DIR}/BuildInfo.gen.hpp" TARGET salvage-headless)
target_include_directories(salvage-headless PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>")
