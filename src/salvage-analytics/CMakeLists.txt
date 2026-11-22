if(NOT TARGET ftxui::ftxui)
    add_subdirectory("${PROJECT_SOURCE_DIR}/external/ftxui" "${PROJECT_BINARY_DIR}/external/ftxui" EXCLUDE_FROM_ALL SYSTEM)
endif()

add_library(salvage-analytics-lib STATIC Dashboard.cpp Report.cpp ReportView.cpp Views.cpp)
add_library(salvage::analytics ALIAS salvage-analytics-lib)
target_sources(salvage-analytics-lib PUBLIC
    FILE_SET HEADERS
    BASE_DIRS "${PROJECT_SOURCE_DIR}/includes"
    FILES "${PROJECT_SOURCE_DIR}/includes/salvage-analytics/Dashboard.hpp"
)
target_link_libraries(salvage-analytics-lib PUBLIC salvage::core PRIVATE ftxui::ftxui nlohmann_json::nlohmann_json csv)
target_compile_definitions(salvage-analytics-lib PRIVATE SALVAGE_ENABLE_THREADS=$<BOOL:${SALVAGE_ENABLE_THREADS}>)
target_set_warnings(salvage-analytics-lib ${ENABLE_WARNINGS_AS_ERRORS})
target_enable_clang_tidy(salvage-analytics-lib)
target_enable_sanitizers(salvage-analytics-lib)
