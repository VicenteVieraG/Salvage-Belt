if(NOT TARGET CLI11::CLI11)
    add_subdirectory("${PROJECT_SOURCE_DIR}/external/cli11" "${PROJECT_BINARY_DIR}/external/cli11" EXCLUDE_FROM_ALL SYSTEM)
endif()

add_library(salvage-headless-lib STATIC Benchmark.cpp Options.cpp Report.cpp)
add_library(salvage::headless ALIAS salvage-headless-lib)
target_sources(salvage-headless-lib PUBLIC
    FILE_SET HEADERS
    BASE_DIRS "${PROJECT_SOURCE_DIR}/includes"
    FILES "${PROJECT_SOURCE_DIR}/includes/salvage-headless/Benchmark.hpp"
)
# The CSV writer is header-only.
target_link_libraries(salvage-headless-lib
    PUBLIC salvage::core
    PRIVATE CLI11::CLI11 nlohmann_json::nlohmann_json $<COMPILE_ONLY:csv>
)
target_set_warnings(salvage-headless-lib ${ENABLE_WARNINGS_AS_ERRORS})
target_enable_clang_tidy(salvage-headless-lib)
target_enable_sanitizers(salvage-headless-lib)

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
configure_file("${PROJECT_SOURCE_DIR}/includes/salvage-headless/BuildInfo.hpp.in"
    "${CMAKE_CURRENT_BINARY_DIR}/salvage-headless/BuildInfo.gen.hpp" @ONLY)
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/salvage-headless/BuildInfo.hpp"
    INPUT "${CMAKE_CURRENT_BINARY_DIR}/salvage-headless/BuildInfo.gen.hpp" TARGET salvage-headless-lib)
target_include_directories(salvage-headless-lib PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>")
