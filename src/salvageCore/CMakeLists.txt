add_library(salvageCore STATIC salvageCore.cpp world/World.cpp)
target_set_warnings(salvageCore ${ENABLE_WARNINGS_AS_ERRORS})
target_enable_clang_tidy(salvageCore)
target_enable_sanitizers(salvageCore)
