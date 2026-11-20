add_library(salvage-core STATIC
    world/World.cpp
    execution/Executor.cpp
    collision/BroadPhase.cpp
    collision/KDTree.cpp
    collision/SweepAndPrune.cpp
    collision/Detection.cpp
    collision/Contact.cpp
    physics/ContactSolver.cpp
    physics/Dynamics.cpp
    simulation/Simulation.cpp
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
