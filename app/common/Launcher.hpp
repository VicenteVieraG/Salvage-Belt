#pragma once

#include <string_view>

namespace salvage::app {
    int launch(int argc, char* argv[], std::string_view name, int (*run)());
}
