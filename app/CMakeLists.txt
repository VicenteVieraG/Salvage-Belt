add_library(salvageAppCommon STATIC common/Simulation.cpp common/Launcher.cpp)
target_include_directories(salvageAppCommon
    PUBLIC common
    PRIVATE ${CMAKE_BINARY_DIR}/configured_files/includes
)
target_link_libraries(salvageAppCommon PUBLIC salvageCore)

add_subdirectory(salvageBelt)
add_subdirectory(salvageAnalytic)

foreach(target IN ITEMS salvageAppCommon salvageBelt salvageAnalytic)
    target_enable_clang_tidy(${target})
    target_enable_sanitizers(${target})
    target_set_warnings(${target} ${ENABLE_WARNINGS_AS_ERRORS})
endforeach()
