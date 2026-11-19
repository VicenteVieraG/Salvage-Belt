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
int main(){
    std::cout<<project_name<<'\n';
    std::cout<<project_version<<'\n';
    std::cout<<"C++ Standard: "<<__cplusplus<<'\n';

    return 0;
}