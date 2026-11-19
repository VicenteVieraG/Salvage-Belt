add_executable(salvageAnalytic main.cpp ${PROJECT_SOURCE_DIR}/src/salvageAnalytic/Tui.cpp)
target_include_directories(salvageAnalytic PRIVATE ${PROJECT_SOURCE_DIR}/includes)
target_link_libraries(salvageAnalytic PRIVATE salvageAppCommon ftxui::component)
