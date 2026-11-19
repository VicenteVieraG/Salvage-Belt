add_library(salvageRaygui OBJECT ${PROJECT_SOURCE_DIR}/src/salvageBelt/RayGui.c)
target_link_libraries(salvageRaygui PRIVATE raylib raygui)

add_executable(salvageBelt main.cpp ${PROJECT_SOURCE_DIR}/src/salvageBelt/Visualizer.cpp)
target_include_directories(salvageBelt PRIVATE ${PROJECT_SOURCE_DIR}/includes)
target_link_libraries(salvageBelt PRIVATE salvageAppCommon raylib raygui salvageRaygui)
