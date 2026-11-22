#include "salvage-analytics/Views.hpp"
#include "salvage-analytics/MouseControls.hpp"

#include <ftxui/component/component.hpp>
#include <ftxui/screen/terminal.hpp>

#include <algorithm>
#include <cmath>
#include <format>
#include <numeric>
#include <optional>

namespace salvage::analytics {
namespace {
using namespace ftxui;

double microseconds(core::Duration time) {
    return std::chrono::duration<double, std::micro>(time).count();
}

Element field(std::string label, std::string value) {
    return hbox({text(std::move(label)), filler(), text(std::move(value)) | bold});
}

Element panel(std::string title, Element content) {
    return window(text(std::move(title)) | color(Color::CyanLight) | bold, std::move(content));
}

Element stepHistory(const std::deque<core::Duration>& history, bool showGraph) {
    if (history.empty()) return text(" Press n for one tick or Space to play.") | dim;
    const double peak = microseconds(std::ranges::max(history));
    const double mean = microseconds(std::accumulate(history.begin(), history.end(), core::Duration{})) / history.size();
    const auto label = std::format(" {} tick{}   Mean {:.1f} µs   Peak {:.1f} µs",
                                   history.size(), history.size() == 1 ? "" : "s", mean, peak);
    if (!showGraph) return text(label);
    auto plot = graph([&history, peak](int width, int height) {
        // Keep oldest on the left and preserve peaks when multiple ticks share a column.
        std::vector<int> bars(static_cast<std::size_t>(width), -1);
        for (std::size_t i = 0; i < history.size() && peak > 0; ++i) {
            const auto slot = historyLimit - history.size() + i;
            const auto column = slot * bars.size() / historyLimit;
            const auto next = std::max(column + 1, (slot + 1) * bars.size() / historyLimit);
            const int level = static_cast<int>(std::ceil(microseconds(history[i]) / peak * height)) - 1;
            for (auto x = column; x < next; ++x) bars[x] = std::max(bars[x], level);
        }
        return bars;
    }) | color(Color::CyanLight) | size(HEIGHT, EQUAL, 3);
    return panel(" Step time ", vbox({text(label), plot}));
}

std::string_view shape(const core::Body& body) {
    return std::holds_alternative<core::SphereCollider>(body.getCollider()) ? "Sphere" : "Box";
}

std::string coordinates(const core::Vec3& v) {
    return std::format("({:.2f}, {:.2f}, {:.2f})", v.x, v.y, v.z);
}

Element bodyDetails(const core::Body& body) {
    return panel(std::format(" Body #{} · {} ", body.getId(), shape(body)), vbox({
        field("Position", coordinates(body.getPosition())), field("Velocity", coordinates(body.getVelocity())),
        field("Inverse mass", std::format("{:.3f}", body.getInverseMass())),
        field("Restitution", std::format("{:.3f}", body.getMaterial().restitution)),
        field("Friction", std::format("{:.3f}", body.getMaterial().friction))
    }));
}

Element bodyList(std::span<const core::Body> bodies, std::size_t& selected, MouseControls& mouse) {
    const auto count = std::min<std::size_t>(bodies.size(), 10);
    const auto first = std::min(selected > count / 2 ? selected - count / 2 : 0, bodies.size() - count);
    Elements rows;
    for (const auto& body : bodies.subspan(first, count)) {
        auto row = text(std::format(" #{:<4} {}", body.getId(), shape(body)));
        rows.push_back(mouse.clickable(body.getId() == selected ? row | inverted : row,
                                       [&selected, id = body.getId()] { selected = id; }));
    }
    return panel(" Bodies ", vbox(std::move(rows))) | size(WIDTH, EQUAL, 18);
}

Element contactList(const core::Simulation& simulation, bool visible) {
    if (!visible) return text("Hidden") | dim;
    if (!simulation.metrics().stepIndex) return text("Waiting for the first tick.") | dim;
    const auto contacts = simulation.contacts();
    if (contacts.empty()) return text("No contacts this tick.") | dim;
    const auto visiblePairs = contacts.first(std::min<std::size_t>(contacts.size(), 3));
    Elements rows;
    for (const auto& contact : visiblePairs)
        rows.push_back(text(std::format("#{} / #{}   depth {:.4f}", contact.first, contact.second, contact.penetration)));
    if (visiblePairs.size() < contacts.size()) rows.push_back(text(std::format("+{} more", contacts.size() - visiblePairs.size())));
    return vbox(std::move(rows));
}

class IndividualsView final : public ComponentBase {
public:
    IndividualsView(const core::Simulation& simulation, std::size_t selected, MouseControls& mouse)
        : simulation_(simulation), selected_(selected), mouse_(mouse) {}

    Element OnRender() override {
        const auto size = Terminal::Size();
        const auto title = text(" Individuals report ") | bold | inverted;
        if (size.dimx < 44 || size.dimy < 20)
            return vbox({title, paragraph("Resize to at least 44 columns and 20 rows."),
                         mouse_.button("Esc Live", Event::Escape), mouse_.button("q Quit", Event::q)});

        const auto bodies = simulation_.world().bodies;
        const auto tick = simulation_.metrics().stepIndex;
        Elements page{hflow({title, text(std::format(" Tick {}   {} {}", tick, bodies.size(),
                                                     bodies.size() == 1 ? "body" : "bodies"))}), separator()};
        if (bodies.empty()) {
            page.push_back(text(" No bodies in this scene."));
        } else {
            updateContacts();
            auto detail = vbox({bodyDetails(bodies[selected_]), panel(" Body contacts ", contacts(tick))});
            auto report = size.dimx >= 68 ? hbox({bodyList(bodies, selected_, mouse_), detail | flex}) : detail;
            page.push_back(mouse_.scrollable(std::move(report), [this](bool forward) { moveBody(forward); }));
        }
        page.push_back(filler());
        page.push_back(separator());
        Elements navigation;
        if (bodies.size() > 1) {
            navigation.push_back(mouse_.button("[ Prev body", Event::Character('[')));
            navigation.push_back(mouse_.button("] Next body", Event::Character(']')));
        }
        if (contacts_.size() > contactsPerPage) {
            navigation.push_back(mouse_.button("← Prev contacts", Event::ArrowLeft));
            navigation.push_back(mouse_.button("→ Next contacts", Event::ArrowRight));
        }
        navigation.push_back(mouse_.button("Esc Live", Event::Escape));
        navigation.push_back(mouse_.button("q Quit", Event::q));
        page.push_back(hflow(std::move(navigation)));
        return vbox(std::move(page));
    }

    bool OnEvent(Event event) override {
        if (event == Event::Character('[') || event == Event::ArrowUp) moveBody(false);
        else if (event == Event::Character(']') || event == Event::ArrowDown) moveBody(true);
        else if (event == Event::ArrowLeft) {
            if (contactOffset_ >= contactsPerPage) contactOffset_ -= contactsPerPage;
        } else if (event == Event::ArrowRight) {
            if (contacts_.size() - contactOffset_ > contactsPerPage) contactOffset_ += contactsPerPage;
        } else return false;
        return true;
    }

private:
    void moveBody(bool forward) {
        const auto count = simulation_.world().bodies.size();
        if (count == 0) return;
        if (forward) selected_ = (selected_ + 1) % count;
        else selected_ = selected_ ? selected_ - 1 : count - 1;
    }

    void updateContacts() {
        if (contactBody_ == selected_) return;
        contacts_.clear();
        for (const auto& contact : simulation_.contacts()) {
            if (contact.first == selected_ || contact.second == selected_) contacts_.push_back(contact);
        }
        contactBody_ = selected_;
        contactOffset_ = 0;
    }

    Element contacts(std::uint64_t tick) const {
        if (tick == 0) return text("Waiting for the first tick.") | dim;
        if (contacts_.empty()) return text("No contacts for this body.") | dim;
        const auto end = contactOffset_ + std::min(contactsPerPage, contacts_.size() - contactOffset_);
        Elements rows{text(std::format("{}–{} of {} contacts", contactOffset_ + 1, end, contacts_.size()))};
        for (auto i = contactOffset_; i < end; ++i) {
            const auto& contact = contacts_[i];
            const auto other = contact.first == selected_ ? contact.second : contact.first;
            rows.push_back(text(std::format("Body #{}   depth {:.4f}", other, contact.penetration)));
        }
        return vbox(std::move(rows));
    }

    static constexpr std::size_t contactsPerPage = 3;
    const core::Simulation& simulation_;
    std::size_t selected_;
    MouseControls& mouse_;
    std::optional<std::size_t> contactBody_;
    std::vector<core::Contact> contacts_;
    std::size_t contactOffset_{};
};
}

ftxui::Element metricsView(const core::SimulationMetrics& m, const std::deque<core::Duration>& history,
                           bool wide, bool showGraph) {
    const auto& d = m.detection;
    auto timeRow = [&](std::string label, core::Duration value) {
        const auto reading = history.empty() ? "—" : std::format("{:.1f}", microseconds(value));
        if (!wide) return field(std::move(label), reading);
        const auto share = !history.empty() && m.totalStep.count() > 0 ? static_cast<float>(value / m.totalStep) : 0.0F;
        return hbox({text(std::move(label)) | size(WIDTH, EQUAL, 14), gauge(share) | flex | color(Color::Cyan),
                     text(std::format("{:>9}", reading))});
    };
    auto countRow = [&](std::string label, auto count) {
        return field(std::move(label), history.empty() ? "—" : std::to_string(count));
    };
    auto timings = panel(" Tick timing (µs) ", vbox({
        timeRow("Total step", m.totalStep), timeRow("Integration", m.integration), timeRow("Detection", d.total),
        timeRow("  Index", d.maintenance), timeRow("  Search", d.search), timeRow("  Normalize", d.normalization),
        timeRow("  Narrow phase", d.narrowPhase), timeRow("Resolution", m.resolution)
    }));
    auto work = panel(" Collision work ", vbox({
        countRow("Examined", d.pairsExamined), countRow("Raw candidates", d.rawCandidates),
        countRow("Unique pairs", d.uniqueCandidates), countRow("Contacts", d.contacts),
        separator(), countRow("Index (B)", d.indexBytes), countRow("Pair buffer (B)", d.pairBufferBytes),
        field("Run peak (B)", std::to_string(m.peakCollisionBufferBytes))
    }));
    return vbox({hbox({timings | flex, work | flex}),
                 hflow({text(std::format(" Generation {:.1f} µs   ", microseconds(m.generation))),
                        text(std::format("Index setup {:.1f} µs", microseconds(m.initialization)))}),
                 stepHistory(history, showGraph)});
}

ftxui::Element inspectorView(const core::Simulation& simulation, std::size_t& selected, bool showContacts, bool wide,
                             MouseControls& mouse) {
    const auto bodies = simulation.world().bodies;
    if (bodies.empty()) return text(" No bodies in this scene.");
    auto inspector = vbox({bodyDetails(bodies[selected]), panel(" Contacts this tick ", contactList(simulation, showContacts))});
    if (wide) inspector = hbox({bodyList(bodies, selected, mouse), inspector | flex});
    return mouse.scrollable(inspector, [&selected, count = bodies.size()](bool forward) {
        if (forward && selected + 1 < count) ++selected;
        else if (!forward && selected > 0) --selected;
    });
}

ftxui::Component individualsView(const core::Simulation& simulation, std::size_t selected, MouseControls& mouse) {
    return ftxui::Make<IndividualsView>(simulation, selected, mouse);
}

}
