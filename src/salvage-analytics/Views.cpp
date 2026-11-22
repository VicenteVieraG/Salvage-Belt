#include "salvage-analytics/Views.hpp"
#include "salvage-analytics/MouseControls.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <numeric>

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
    const auto& body = bodies[selected];
    auto details = panel(std::format(" Body #{} · {} ", body.getId(), shape(body)), vbox({
        field("Position", coordinates(body.getPosition())), field("Velocity", coordinates(body.getVelocity())),
        field("Inverse mass", std::format("{:.3f}", body.getInverseMass())),
        field("Restitution", std::format("{:.3f}", body.getMaterial().restitution)),
        field("Friction", std::format("{:.3f}", body.getMaterial().friction))
    }));
    auto inspector = vbox({details, panel(" Contacts this tick ", contactList(simulation, showContacts))});
    if (wide) inspector = hbox({bodyList(bodies, selected, mouse), inspector | flex});
    return mouse.scrollable(inspector, [&selected, count = bodies.size()](bool forward) {
        if (forward && selected + 1 < count) ++selected;
        else if (!forward && selected > 0) --selected;
    });
}

}
