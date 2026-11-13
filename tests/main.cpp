/**
 * @file
 * @brief Catch2 entry point and sample factorial tests.
 */
#include <catch2/catch_test_macros.hpp>
#include <cstdint>

#ifdef SALVAGE_TESTS_CUSTOM_MAIN
#include <catch2/catch_session.hpp>

/**
 * @brief Run Catch2 with the optional custom test entry point.
 * @param argc Number of command-line arguments.
 * @param argv Arguments forwarded to Catch2.
 * @return The Catch2 session exit status.
 */
int main(int argc, char* argv[]) {
    return Catch::Session().run(argc, argv);
}
#endif

/**
 * @brief Compute the recursive product used by the sample tests.
 * @param number Non-negative integer whose product is requested.
 * @return The factorial for positive inputs, or zero when number is zero.
 * @note Arithmetic uses unsigned 32-bit values and can overflow for large inputs.
 */
std::uint32_t factorial( std::uint32_t number ) {
    return number <= 1 ? number : factorial(number-1) * number;
}

/**
 * @brief Factorials are computed.
 * @test Factorials are computed.
 */
TEST_CASE( "Factorials are computed", "[factorial]" ) {
    REQUIRE( factorial( 1) == 1 );
    REQUIRE( factorial( 2) == 2 );
    REQUIRE( factorial( 3) == 6 );
    REQUIRE( factorial(10) == 3'628'800 );
}
