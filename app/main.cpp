/**
 * @file
 * @brief Application entry point.
 */
#include <iostream>
#include <config.hpp>

/**
 * @brief Print project metadata and the active C++ language standard.
 * @param argc Number of command-line arguments (unused).
 * @param argv Command-line arguments (unused).
 * @return Zero on normal completion.
 */
int main(int argc, char* argv[]){
    std::cout<<project_name<<std::endl;
    std::cout<<project_version<<std::endl;
    std::cout<<"C++ Standard: "<<__cplusplus<<std::endl;

    return 0;
}