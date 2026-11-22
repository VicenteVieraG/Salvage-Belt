#include "salvage-analytics/Dashboard.hpp"
#include "salvage-analytics/BodyCount.hpp"
#include "salvage-analytics/MouseControls.hpp"
#include "salvage-analytics/ReportView.hpp"
#include "salvage-analytics/Views.hpp"

#include <ftxui/component/app.hpp>
#include <ftxui/component/component.hpp>
#include <ftxui/component/event.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <format>
#include <stdexcept>
#include <thread>
#include <utility>

#ifdef _WIN32
#include <io.h>
#else
#include <unistd.h>
#endif

namespace salvage::analytics {
namespace {
using namespace ftxui;
using core::Algorithm;

constexpr std::array algorithms{Algorithm::bruteForce, Algorithm::kdTree, Algorithm::sweepAndPrune};
constexpr std::array algorithmNames{"Brute force", "k-d tree", "Sweep and prune"};

bool hasTerminal() {
#ifdef _WIN32
    return _isatty(0) && _isatty(1);
#else
    return isatty(STDIN_FILENO) && isatty(STDOUT_FILENO);
#endif
}

class Dashboard final : public ComponentBase {
public:
    Dashboard(App& app, MouseControls& mouse, core::SimulationConfig config)
        : app_(app), mouse_(mouse), simulation_(std::move(config)) {
        if (const auto* field = std::get_if<core::FieldSettings>(&simulation_.config().scene))
            referenceField_ = *field;
    }

    void pause() { playing_ = false; elapsed_ = 0; }

    std::size_t bodyCount() const { return simulation_.world().bodies.size(); }

    void setBodyCount(core::BodyId count) {
        if (count == bodyCount()) return;
        auto config = simulation_.config();
        config.scene = fieldWithBodyCount(count, referenceField_);
        simulation_.reset(std::move(config));
        selected_ = count ? std::min<std::size_t>(selected_, count - 1) : 0;
        history_.clear();
        error_.clear();
    }

    Element OnRender() override {
        const auto& config = simulation_.config();
        const auto metrics = simulation_.metrics();
        const auto size = Terminal::Size();
        const bool wide = size.dimx >= 68;
        if (size.dimx < 44 || size.dimy < 20)
            return vbox({text("Salvage Analytics") | bold, paragraph("Resize to at least 44 columns and 20 rows."),
                         mouse_.button("q Quit", Event::q)});

        auto heading = hbox({
            text(" Salvage Analytics ") | bold,
            filler(),
            text(playing_ ? " Running " : " Paused ") | inverted,
            text(wide ? std::format("  Tick {}   {:.3f} s ", metrics.stepIndex, metrics.stepIndex * config.fixedStep)
                      : std::format("  Tick {} ", metrics.stepIndex))
        });
        Elements settings;
        for (std::size_t i = 0; i < algorithms.size(); ++i) {
            auto choice = text(std::format(" {} {}{}", i + 1, algorithmNames[i], wide ? " " : ""));
            settings.push_back(mouse_.clickable(choice | (config.algorithm == algorithms[i] ? inverted : underlined),
                                                [this, i] { switchAlgorithm(algorithms[i]); }));
        }
        const auto count = bodyCount();
        settings.push_back(mouse_.button(std::format("b {} {}", count, count == 1 ? "body" : "bodies"), Event::b));
        settings.push_back(text(std::format(" {} worker{}  ", config.workers, config.workers == 1 ? "" : "s")));
        settings.push_back(text(std::holds_alternative<core::FieldSettings>(config.scene)
                                    ? std::format("Seed {}", config.seed) : "Custom scene"));
        auto tabs = hbox({
            mouse_.clickable(text(" Metrics ") | (inspect_ ? underlined : inverted), [this] { setInspector(false); }),
            mouse_.clickable(text(" Inspector ") | (inspect_ ? inverted : underlined), [this] { setInspector(true); })
        });
        Elements commands{mouse_.button(playing_ ? "Space Pause" : "Space Play", Event::Character(' ')),
                          mouse_.button("n Step", Event::n), mouse_.button("r Reset", Event::r)};
        if (wide) commands.push_back(mouse_.button("Tab View", Event::Tab));
        commands.push_back(mouse_.button(wide ? "o Open report" : "o Open", Event::o));
        if (wide) commands.push_back(mouse_.button("q Quit", Event::q));
        Elements page{heading, hflow(std::move(settings)), separator(), tabs,
                      inspect_ ? inspectorView(simulation_, selected_, showContacts_, wide, mouse_)
                               : metricsView(metrics, history_, wide, size.dimy >= (wide ? 24 : 26)),
                      filler(), separator(), hflow(std::move(commands))};
        Elements shortcuts;
        if (!wide) {
            shortcuts.push_back(mouse_.button("Tab", Event::Tab));
            shortcuts.push_back(mouse_.button("q Quit", Event::q));
        }
        if (inspect_) {
            shortcuts.push_back(mouse_.button("[ Prev", Event::Character('[')));
            shortcuts.push_back(mouse_.button("] Next", Event::Character(']')));
            shortcuts.push_back(mouse_.button("c Contacts", Event::c));
        } else {
            shortcuts.push_back(mouse_.button(wide ? "a Next algorithm" : "a Algorithm", Event::a));
#if SALVAGE_ENABLE_THREADS
            shortcuts.push_back(mouse_.button(wide ? "w Change workers" : "w Workers", Event::w));
#endif
        }
        page.push_back(hflow(std::move(shortcuts)));
        if (!error_.empty()) page.push_back(paragraph(error_) | color(Color::RedLight));
        return vbox(std::move(page));
    }

    bool OnEvent(Event event) override {
        if (event == Event::Escape) app_.Exit();
        else if (event == Event::Tab || event == Event::TabReverse) setInspector(!inspect_);
        else if (event.is_character() && event.character().size() == 1) {
            switch (std::tolower(static_cast<unsigned char>(event.character()[0]))) {
                case 'q': app_.Exit(); break;
                case ' ':
                    playing_ = !playing_;
                    elapsed_ = 0;
                    if (playing_) app_.RequestAnimationFrame();
                    break;
                case 'n':
                    playing_ = false;
                    step();
                    break;
                case 'r': restart(simulation_.config().workers); break;
                case 'a': switchAlgorithm(algorithms[(algorithmIndex() + 1) % algorithms.size()]); break;
                case '1': case '2': case '3': switchAlgorithm(algorithms[event.character()[0] - '1']); break;
#if SALVAGE_ENABLE_THREADS
                case 'w': {
                    const auto count = simulation_.config().workers;
                    const auto limit = std::max(1u, std::thread::hardware_concurrency());
                    restart(count >= limit ? 1 : std::min<std::size_t>(count * 2, limit));
                    break;
                }
#endif
                case '[': selectBody(false); break;
                case ']': selectBody(true); break;
                case 'c':
                    setInspector(true);
                    showContacts_ = !showContacts_;
                    break;
                default: return false;
            }
        } else return false;
        return true;
    }

    void OnAnimation(animation::Params& params) override {
        if (!playing_) return;
        const auto fixedStep = simulation_.config().fixedStep;
        elapsed_ += params.duration().count();
        // Drop excessive backlog after a slow frame so input stays responsive.
        elapsed_ = std::min(elapsed_, 8 * fixedStep);
        while (playing_ && elapsed_ >= fixedStep) {
            elapsed_ -= fixedStep;
            step();
            if (simulation_.metrics().totalStep.count() >= fixedStep) {
                elapsed_ = 0;
                break; // Let the terminal process input between expensive ticks.
            }
        }
        if (playing_) app_.RequestAnimationFrame();
    }

private:
    void setInspector(bool inspect) {
        if (inspect_ != inspect) mouse_.clear();
        inspect_ = inspect;
    }

    std::size_t algorithmIndex() const {
        return static_cast<std::size_t>(std::ranges::find(algorithms, simulation_.config().algorithm) - algorithms.begin());
    }

    void switchAlgorithm(Algorithm algorithm) {
        if (algorithm == simulation_.config().algorithm) return;
        try {
            simulation_.setAlgorithm(algorithm);
            history_.clear();
            error_.clear();
        } catch (const std::exception& error) {
            error_ = error.what();
        }
    }

    void restart(std::size_t workers) {
        pause();
        try {
            simulation_.restartWith(simulation_.config().algorithm, workers);
            history_.clear();
            error_.clear();
        } catch (const std::exception& error) {
            error_ = error.what();
        }
    }

    void step() {
        try {
            simulation_.step();
            if (history_.size() == historyLimit) history_.pop_front();
            history_.push_back(simulation_.metrics().totalStep);
            error_.clear();
        } catch (const std::exception& error) {
            playing_ = false;
            error_ = error.what();
        }
    }

    void selectBody(bool forward) {
        setInspector(true);
        const auto count = simulation_.world().bodies.size();
        if (count == 0) return;
        if (forward) selected_ = (selected_ + 1) % count;
        else selected_ = selected_ ? selected_ - 1 : count - 1;
    }

    App& app_;
    MouseControls& mouse_;
    core::Simulation simulation_;
    core::FieldSettings referenceField_;
    std::deque<core::Duration> history_;
    std::size_t selected_{};
    bool inspect_{}, showContacts_{true}, playing_{};
    double elapsed_{};
    std::string error_;
};

void runApp(core::SimulationConfig config, std::optional<SavedReport> initialReport) {
    if (!hasTerminal()) throw std::runtime_error("an interactive terminal is required");
    auto app = ftxui::App::Fullscreen();
    app.TrackMouse(true);
    Component shortcuts;
    MouseControls mouse([&shortcuts](Event key) { shortcuts->OnEvent(std::move(key)); });
    auto host = Make<ComponentBase>();
    auto show = [&](const Component& page) {
        mouse.clear();
        host->DetachAllChildren();
        host->Add(page);
    };
    std::shared_ptr<Dashboard> live;
    Component current;
    auto showLive = [&] {
        if (!live) live = Make<Dashboard>(app, mouse, std::move(config));
        current = live;
        show(current);
    };
    if (initialReport) {
        current = reportView(std::move(*initialReport), mouse);
        show(current);
    } else showLive();

    enum class Prompt { none, report, bodies };
    auto editing = Prompt::none;
    std::string entry, placeholder, error;
    InputOption options;
    options.multiline = false;
    auto input = Input(&entry, &placeholder, options);
    auto prompt = Renderer(input, [&] {
        const bool bodies = editing == Prompt::bodies;
        return vbox({text(bodies ? " Set body count " : " Open saved benchmark ") | bold, separator(),
                     bodies ? paragraph("Changing the count starts a new seeded field at tick zero.") : text(" Report file"),
                     input->Render() | border, paragraph(error) | color(Color::RedLight),
                     filler(), hbox({mouse.button(bodies ? "Enter Apply" : "Enter Open", Event::Return),
                                     mouse.button("Esc Cancel", Event::Escape)})});
    });
    shortcuts = CatchEvent(host, [&](Event event) {
        if (editing != Prompt::none) {
            if (event == Event::Return) {
                try {
                    if (editing == Prompt::bodies) live->setBodyCount(parseBodyCount(entry));
                    else current = reportView(loadReport(entry), mouse);
                } catch (const std::exception& e) {
                    error = displayText(e.what());
                    return true;
                }
            } else if (event != Event::Escape) return false;
            editing = Prompt::none;
            show(current);
            return true;
        }
        const bool setCount = current == live && (event == Event::b || event == Event::B);
        if (setCount || event == Event::o || event == Event::O) {
            if (live) live->pause();
            editing = setCount ? Prompt::bodies : Prompt::report;
            placeholder = setCount ? std::format("Number of bodies (currently {})", live->bodyCount())
                                   : "Path to a JSON or CSV report";
            entry.clear();
            error.clear();
            show(prompt);
            return true;
        }
        if (current != live && event == Event::Escape) {
            if (!current->OnEvent(event)) showLive();
            return true;
        }
        if (event == Event::q || event == Event::Q) {
            app.Exit();
            return true;
        }
        return false;
    });
    auto page = Renderer(shortcuts, [&] {
        mouse.clear();
        return shortcuts->Render();
    });
    auto root = CatchEvent(page, [&](Event event) { return event.is_mouse() && mouse.handle(event.mouse()); });
    app.PostEvent(ftxui::Event::Custom);
    app.Loop(root);
}
}

void runDashboard(core::SimulationConfig config) { runApp(std::move(config), {}); }

void runReport(const std::filesystem::path& path) { runApp({}, loadReport(path)); }

}
