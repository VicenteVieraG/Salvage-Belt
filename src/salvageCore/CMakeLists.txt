add_library(salvageCore STATIC
    salvageCore.cpp
    world/World.cpp
    physics/Contact.cpp
    physics/ContactSolver.cpp
    algorithm/BroadPhaseFactory.cpp
    algorithm/BruteForceBroadPhase.cpp
    algorithm/KDTreeBroadPhase.cpp
    algorithm/SweepAndPruneBroadPhase.cpp
)
find_package(Threads REQUIRED)
target_link_libraries(salvageCore PRIVATE Threads::Threads)
target_set_warnings(salvageCore OFF)
target_enable_clang_tidy(salvageCore)
target_enable_sanitizers(salvageCore)
