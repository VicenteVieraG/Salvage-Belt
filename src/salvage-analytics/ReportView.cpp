#include "ReportView.hpp"
#include "MouseControls.hpp"

#include <ftxui/component/component.hpp>
#include <ftxui/component/event.hpp>
#include <ftxui/screen/terminal.hpp>

#include <algorithm>
#include <format>

namespace salvage::analytics {
namespace {
using namespace ftxui;

std::string micros(double seconds) { return std::format("{:.2f}", seconds * 1e6); }

std::string comparisonRow(bool wide, std::string algorithm, std::string workers, std::string median,
                          std::string p95, std::string speedup, std::string status) {
    if (wide) return std::format(" {:<16} {:>7} {:>11} {:>11} {:>9}  {}", algorithm, workers, median, p95, speedup, status);
    return std::format(" {:<15} {:>3} {:>10} {:>10}", algorithm, workers, median, p95);
}

bool move(std::size_t& index, std::size_t count, bool forward) {
    if (forward && index + 1 < count) ++index;
    else if (!forward && index > 0) --index;
    else return false;
    return true;
}

class ReportView final : public ComponentBase {
public:
    ReportView(SavedReport report, MouseControls& mouse) : report_(std::move(report)), mouse_(mouse) {}

    Element OnRender() override {
        const auto size = Terminal::Size();
        if (size.dimx < 44 || size.dimy < 20)
            return vbox({text("Saved benchmark") | bold, paragraph("Resize to at least 44 columns and 20 rows."),
                         mouse_.button("q Quit", Event::q)});
        const auto& p = report_.protocol;
        const auto& scene = currentCase().scene;
        Elements page{
            hbox({text(" Saved benchmark ") | bold | inverted, text(" " + report_.filename) | flex}),
            hflow({text(std::format(" Case {}/{}   {}   ", case_ + 1, report_.cases.size(), displayText(scene.scenario))),
                   text(std::format("{} bodies   Seed {}", scene.bodies, scene.seed))}),
            hflow({text(p.replay ? " Replay · detection time   " : " Simulation · total step time   "),
                   text(std::format("{} warmup   {} ticks × {} reps", p.warmup, p.steps, p.repetitions))}),
            separator(),
            mouse_.scrollable(vbox({detail_ ? details() : comparison(size.dimx >= 76, std::max(1, size.dimy - 14)),
                                    filler()}) | flex, [this](bool forward) { moveVertical(forward); }),
            separator()
        };
        Elements navigation{
            hbox({text(detail_ ? " Tick" : " Variant"),
                  mouse_.button("↑", Event::ArrowUp), mouse_.button("↓", Event::ArrowDown)}),
            hbox({text(detail_ ? " Repetition" : " Case"),
                  mouse_.button("←", Event::ArrowLeft), mouse_.button("→", Event::ArrowRight)})
        };
        if (!detail_) navigation.push_back(mouse_.button("Enter Details", Event::Return));
        navigation.push_back(mouse_.button(detail_ ? "Esc Back" : "Esc Live", Event::Escape));
        page.push_back(hflow(std::move(navigation)));
        page.push_back(hbox({mouse_.button("o Open report", Event::o), mouse_.button("q Quit", Event::q)}));
        return vbox(std::move(page));
    }

    bool OnEvent(Event event) override {
        if (event == Event::Return || (detail_ && event == Event::Escape)) {
            mouse_.clear();
            detail_ = !detail_;
            run_ = tick_ = 0;
            return true;
        }
        const bool horizontal = event == Event::ArrowLeft || event == Event::ArrowRight;
        const bool vertical = event == Event::ArrowUp || event == Event::ArrowDown;
        if (!horizontal && !vertical) return false;
        const bool forward = event == Event::ArrowRight || event == Event::ArrowDown;
        if (vertical) {
            moveVertical(forward);
        } else if (!detail_) {
            if (move(case_, report_.cases.size(), forward)) {
                mouse_.clear();
                variant_ = 0;
            }
        } else if (move(run_, variant().runs.size(), forward)) tick_ = 0;
        return true;
    }

private:
    void moveVertical(bool forward) {
        if (!detail_) move(variant_, currentCase().variants.size(), forward);
        else if (!variant().runs.empty()) move(tick_, variant().runs[run_].samples.size(), forward);
    }

    const ReportCase& currentCase() const { return report_.cases[case_]; }
    const ReportVariant& variant() const { return currentCase().variants[variant_]; }

    Element comparison(bool wide, std::size_t capacity) {
        const auto& variants = currentCase().variants;
        const auto count = std::min(capacity, variants.size());
        const auto first = std::min(variant_ > count / 2 ? variant_ - count / 2 : 0, variants.size() - count);
        Elements rows{text(" Repetition means · µs per tick") | color(Color::CyanLight),
                      text(comparisonRow(wide, "Algorithm", wide ? "Workers" : "CPU", "Median", "p95", "vs brute", "Status")) | bold};
        for (auto i = first; i < first + count; ++i) {
            const auto& v = variants[i];
            auto row = text(comparisonRow(wide, displayText(v.algorithm), std::to_string(v.workers),
                v.timing ? micros(v.timing->median) : "—", v.timing ? micros(v.timing->p95) : "—",
                v.speedupVsBrute ? std::format("{:.2f}×", *v.speedupVsBrute) : "—",
                v.status == "failed" ? "failed" : displayText(v.validation)));
            rows.push_back(mouse_.clickable(i == variant_ ? row | inverted : row, [this, i] { variant_ = i; }));
        }
        rows.push_back(separator());
        rows.push_back(hflow({text(std::format(" Variant {}/{}   ", variant_ + 1, variants.size())),
                              text(std::format("{} · validation {}", variant().status, displayText(variant().validation)))}));
        if (variant().status == "failed") rows.push_back(paragraph(displayText(variant().error)) | color(Color::RedLight));
        return vbox(std::move(rows));
    }

    Element details() const {
        const auto& v = variant();
        const auto& scene = currentCase().scene;
        const auto& p = report_.protocol;
        Elements rows{text(std::format(" {}   {} worker{}", displayText(v.algorithm), v.workers, v.workers == 1 ? "" : "s")) | bold,
                      text(std::format(" {} · validation {}", v.status, displayText(v.validation))),
                      hflow({text(std::format(" {} spheres, {} boxes   ", scene.asteroids, scene.debris)),
                             text(std::format("{}   ", displayText(p.response))),
                             text(std::format("Fixed {}   ", displayText(p.scale))),
                             text(std::format("Step {:.3g} ms", 1e3 * p.fixedStep))})};
        if (v.timing) {
            const auto& t = *v.timing;
            rows.push_back(hflow({text(std::format(" Mean {} µs   ", micros(t.mean))),
                                  text(std::format("Median {} µs   ", micros(t.median))),
                                  text(std::format("p95 {} µs", micros(t.p95)))}));
            rows.push_back(text(v.speedupVsBrute ? std::format(" {:.2f}× vs one-worker brute force", *v.speedupVsBrute)
                                                 : " Speedup vs one-worker brute force: —"));
        } else {
            rows.push_back(paragraph(displayText(v.error)) | color(Color::RedLight));
        }
        rows.push_back(separator());
        if (v.runs.empty()) {
            rows.push_back(text(" No measured ticks were saved."));
        } else {
            const auto& run = v.runs[run_];
            const auto& s = run.samples[tick_];
            rows.push_back(text(std::format(" Repetition {}/{}   Tick {}   ({}/{})", run.repetition,
                report_.protocol.repetitions, s.tick, tick_ + 1, run.samples.size())) | color(Color::CyanLight));
            if (!report_.protocol.replay) rows.push_back(text(" Total step    " + micros(s.seconds) + " µs"));
            rows.push_back(text(" Detection     " + micros(s.detectionSeconds) + " µs"));
            rows.push_back(text(std::format(" Examined {}   Candidates {}   Contacts {}", s.examined, s.candidates, s.contacts)));
        }
        return vbox(std::move(rows));
    }

    SavedReport report_;
    MouseControls& mouse_;
    std::size_t case_{}, variant_{}, run_{}, tick_{};
    bool detail_{};
};
}

ftxui::Component reportView(SavedReport report, MouseControls& mouse) {
    return ftxui::Make<ReportView>(std::move(report), mouse);
}

}
