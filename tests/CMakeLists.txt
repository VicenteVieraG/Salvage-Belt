if(BUILD_TESTS)
    set(CATCH2_DIR "${CMAKE_CURRENT_SOURCE_DIR}/catch2")

    if(EXISTS "${CATCH2_DIR}/CMakeLists.txt")
        add_subdirectory(${CATCH2_DIR})
        add_executable(tests ContactTests.cpp DetectionTests.cpp SimulationTests.cpp)
        target_link_libraries(tests PRIVATE salvage::core)
        target_set_warnings(tests ${ENABLE_WARNINGS_AS_ERRORS})
        target_enable_clang_tidy(tests)
        target_enable_sanitizers(tests)
        target_compile_definitions(tests PRIVATE SALVAGE_ENABLE_THREADS=$<BOOL:${SALVAGE_ENABLE_THREADS}>)

        if(BUILD_TESTS_WITH_MAIN)
            target_sources(tests PRIVATE main.cpp)
            target_link_libraries(tests PUBLIC Catch2::Catch2)
        else()
            target_link_libraries(tests PUBLIC Catch2::Catch2WithMain)
        endif()

        include("${CATCH2_DIR}/extras/Catch.cmake")
        catch_discover_tests(tests)
    else()
        message(FATAL_ERROR
            "Catch2 submodule not found at ${CATCH2_DIR}. "
            "Run: git submodule update --init --recursive"
        )
    endif()
endif()

if(BUILD_TESTS AND TARGET salvage-analytics)
    add_dependencies(tests salvage-analytics)
    add_test(NAME salvage-analytics.help COMMAND salvage-analytics --help)
    if(UNIX)
        find_package(Python3 QUIET COMPONENTS Interpreter)
        if(Python3_Interpreter_FOUND)
            add_test(NAME salvage-analytics.terminal COMMAND "${Python3_EXECUTABLE}"
                "${CMAKE_CURRENT_SOURCE_DIR}/AnalyticsTerminal.py" "$<TARGET_FILE:salvage-analytics>")
            set_tests_properties(salvage-analytics.terminal PROPERTIES TIMEOUT 20)
            if(TARGET salvage-headless)
                add_test(NAME salvage-analytics.reports COMMAND "${Python3_EXECUTABLE}" -B
                    "${CMAKE_CURRENT_SOURCE_DIR}/AnalyticsReports.py" "$<TARGET_FILE:salvage-analytics>"
                    "$<TARGET_FILE:salvage-headless>" "${SALVAGE_ENABLE_THREADS}")
                set_tests_properties(salvage-analytics.reports PROPERTIES TIMEOUT 45)
            endif()
        endif()
    endif()
endif()

if(BUILD_TESTS AND TARGET salvage-headless)
    add_dependencies(tests salvage-headless)
    foreach(mode simulation replay)
        add_test(NAME "salvage-headless.${mode}" COMMAND "${CMAKE_COMMAND}"
            "-DCLI=$<TARGET_FILE:salvage-headless>" "-DMODE=${mode}"
            "-DTHREADED=${SALVAGE_ENABLE_THREADS}" -P "${CMAKE_CURRENT_SOURCE_DIR}/Headless.cmake")
    endforeach()
endif()
