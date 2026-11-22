#pragma once

#include <ftxui/component/event.hpp>
#include <ftxui/dom/elements.hpp>
#include <ftxui/screen/box.hpp>

#include <deque>
#include <functional>
#include <string>
#include <utility>

namespace salvage::analytics {

// Register just the controls in the current layout. FTXUI fills in their actual
// terminal bounds during rendering, including any clipping and line wrapping.
class MouseControls {
public:
    explicit MouseControls(std::function<void(ftxui::Event)> dispatch) : dispatch_(std::move(dispatch)) {}

    void clear() { clicks_.clear(); scrolls_.clear(); }
    ftxui::Element button(std::string label, ftxui::Event shortcut);
    ftxui::Element clickable(ftxui::Element element, std::function<void()> click);
    ftxui::Element scrollable(ftxui::Element element, std::function<void(bool forward)> scroll);
    bool handle(const ftxui::Mouse& mouse) const;

private:
    template<class Handler>
    struct Target {
        ftxui::Box box{0, -1, 0, -1};
        Handler activate;
    };
    // reflect() borrows each box until the layout is drawn.
    std::deque<Target<std::function<void()>>> clicks_;
    std::deque<Target<std::function<void(bool)>>> scrolls_;
    std::function<void(ftxui::Event)> dispatch_;
};

}
