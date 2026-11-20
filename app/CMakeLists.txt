add_executable(${CMAKE_PROJECT_NAME} main.cpp)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE salvage::core)
target_enable_clang_tidy(${CMAKE_PROJECT_NAME})
target_enable_sanitizers(${CMAKE_PROJECT_NAME})

target_include_directories(${CMAKE_PROJECT_NAME}
    PRIVATE ./
    PRIVATE ${CMAKE_BINARY_DIR}/configured_files/includes
)

target_set_warnings(
    ${CMAKE_PROJECT_NAME}
    ${ENABLE_WARNINGS_AS_ERRORS}
)

option(SALVAGE_BUILD_HEADLESS "Build the salvage-headless benchmark CLI." ON)
if(SALVAGE_BUILD_HEADLESS)
    add_subdirectory(headless)
endif()
