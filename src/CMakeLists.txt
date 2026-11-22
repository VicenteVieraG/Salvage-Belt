add_library(salvage-core STATIC
    salvage-core/world/World.cpp
    salvage-core/execution/Executor.cpp
    salvage-core/collision/BroadPhase.cpp
    salvage-core/collision/KDTree.cpp
    salvage-core/collision/SweepAndPrune.cpp
    salvage-core/collision/Detection.cpp
    salvage-core/collision/Contact.cpp
    salvage-core/physics/ContactSolver.cpp
    salvage-core/physics/Dynamics.cpp
    salvage-core/simulation/Simulation.cpp
)
add_library(salvage::core ALIAS salvage-core)
target_include_directories(salvage-core PUBLIC "${PROJECT_SOURCE_DIR}/includes")
target_compile_features(salvage-core PUBLIC cxx_std_26)
target_compile_definitions(salvage-core PRIVATE SALVAGE_ENABLE_THREADS=$<BOOL:${SALVAGE_ENABLE_THREADS}>)
if(SALVAGE_ENABLE_THREADS)
    find_package(Threads REQUIRED)
    target_link_libraries(salvage-core PRIVATE Threads::Threads)
endif()
target_set_warnings(salvage-core ${ENABLE_WARNINGS_AS_ERRORS})
target_enable_clang_tidy(salvage-core)
target_enable_sanitizers(salvage-core)

if(SALVAGE_BUILD_HEADLESS OR SALVAGE_BUILD_ANALYTICS)
    if(NOT TARGET nlohmann_json::nlohmann_json)
        add_subdirectory("${PROJECT_SOURCE_DIR}/external/json" "${PROJECT_BINARY_DIR}/external/json" EXCLUDE_FROM_ALL SYSTEM)
    endif()
    if(NOT TARGET csv)
        set(CSV_BUILD_PROGRAMS OFF)
        set(CSV_ENABLE_THREADS OFF)
        add_subdirectory("${PROJECT_SOURCE_DIR}/external/csv" "${PROJECT_BINARY_DIR}/external/csv" EXCLUDE_FROM_ALL SYSTEM)
    endif()
endif()

if(SALVAGE_BUILD_HEADLESS)
    add_subdirectory(salvage-headless)
endif()

if(SALVAGE_BUILD_ANALYTICS)
    add_subdirectory(salvage-analytics)
endif()
