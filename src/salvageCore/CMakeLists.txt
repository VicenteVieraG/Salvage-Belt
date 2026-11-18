add_library(salvageCore STATIC
    salvageCore.cpp
    world/World.cpp
    algorithm/BroadPhaseFactory.cpp
    algorithm/BruteForceBroadPhase.cpp
    algorithm/KDTreeBroadPhase.cpp
)
# target_set_warnings(salvageCore)
target_enable_clang_tidy(salvageCore)
target_enable_sanitizers(salvageCore)
