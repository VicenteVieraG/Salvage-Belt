#include "MouseControls.hpp"
#include <algorithm>

namespace salvage::analytics {
using namespace ftxui;

Element MouseControls::button(std::string label, Event shortcut) {
    auto control = hbox({text(" "), text(std::move(label)) | underlined, text(" ")});
    return clickable(std::move(control), [this, shortcut] { dispatch_(shortcut); });
}

Element MouseControls::clickable(Element element, std::function<void()> click) {
    auto& target = clicks_.emplace_back();
    target.activate = std::move(click);
    return std::move(element) | reflect(target.box);
}

Element MouseControls::scrollable(Element element, std::function<void(bool)> scroll) {
    auto& target = scrolls_.emplace_back();
    target.activate = std::move(scroll);
    return std::move(element) | reflect(target.box);
}

bool MouseControls::handle(const Mouse& mouse) const {
    if (mouse.motion != Mouse::Pressed) return false;
    auto contains = [&](const auto& target) { return target.box.Contain(mouse.x, mouse.y); };
    if (mouse.button == Mouse::Left) {
        auto hit = std::ranges::find_if(clicks_, contains);
        if (hit == clicks_.end()) return false;
        auto click = hit->activate; // Changing pages can discard this target.
        click();
        return true;
    }
    if (mouse.button == Mouse::WheelUp || mouse.button == Mouse::WheelDown) {
        auto hit = std::ranges::find_if(scrolls_, contains);
        if (hit == scrolls_.end()) return false;
        auto scroll = hit->activate;
        scroll(mouse.button == Mouse::WheelDown);
        return true;
    }
    return false;
}

}
