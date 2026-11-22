#pragma once

#include "salvage-analytics/Report.hpp"
#include <ftxui/component/component_base.hpp>

namespace salvage::analytics {

class MouseControls;
ftxui::Component reportView(SavedReport report, MouseControls& mouse);

}
