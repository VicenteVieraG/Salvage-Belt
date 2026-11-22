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

if(SALVAGE_BUILD_HEADLESS)
    add_executable(salvage-headless headless_main.cpp)
    target_link_libraries(salvage-headless PRIVATE salvage::headless)
    target_set_warnings(salvage-headless ${ENABLE_WARNINGS_AS_ERRORS})
    target_enable_clang_tidy(salvage-headless)
    target_enable_sanitizers(salvage-headless)
endif()

if(SALVAGE_BUILD_ANALYTICS)
    add_executable(salvage-analytics analytics_main.cpp)
    target_link_libraries(salvage-analytics PRIVATE salvage::analytics)
    target_include_directories(salvage-analytics PRIVATE "${CMAKE_BINARY_DIR}/configured_files/includes")
    target_set_warnings(salvage-analytics ${ENABLE_WARNINGS_AS_ERRORS})
    target_enable_clang_tidy(salvage-analytics)
    target_enable_sanitizers(salvage-analytics)
endif()
