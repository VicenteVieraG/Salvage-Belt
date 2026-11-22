#include "salvage-analytics/Dashboard.hpp"
#include "ReportView.hpp"
#include "Views.hpp"

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
    Dashboard(App& app, core::SimulationConfig config) : app_(app), simulation_(std::move(config)) {}

    void pause() { playing_ = false; elapsed_ = 0; }

    Element OnRender() override {
        const auto& config = simulation_.config();
        const auto metrics = simulation_.metrics();
        const auto size = Terminal::Size();
        const bool wide = size.dimx >= 68;
        if (size.dimx < 44 || size.dimy < 20)
            return vbox({text("Salvage Analytics") | bold, paragraph("Resize to at least 44 columns and 20 rows."),
                         text("q  Quit")});

        auto heading = hbox({
            text(" Salvage Analytics ") | bold,
            filler(),
            text(playing_ ? " Running " : " Paused ") | inverted,
            text(wide ? std::format("  Tick {}   {:.3f} s ", metrics.stepIndex, metrics.stepIndex * config.fixedStep)
                      : std::format("  Tick {} ", metrics.stepIndex))
        });
        auto settings = hflow({
            text(std::format(" {}   ", algorithmNames[algorithmIndex()])),
            text(std::format("{} worker{}   ", config.workers, config.workers == 1 ? "" : "s")),
            text(std::format("{} bodies   ", simulation_.world().bodies.size())),
            text(std::holds_alternative<core::FieldSettings>(config.scene) ? std::format("Seed {}", config.seed)
                                                                          : "Custom scene")
        });
        auto tabs = hbox({text(" Metrics ") | (inspect_ ? dim : inverted),
                          text(" Inspector ") | (inspect_ ? inverted : dim)});
        auto commands = text(wide ? " Space Play/Pause   n Step   r Reset   Tab View   o Open report   q Quit"
                                  : " Space Play/Pause  n Step  r Reset  o Open");
        Elements page{heading, settings, separator(), tabs,
                      inspect_ ? inspectorView(simulation_, selected_, showContacts_, wide)
                               : metricsView(metrics, history_, wide, size.dimy >= (wide ? 24 : 26)),
                      filler(), separator(), commands};
        std::string shortcuts = wide ? " " : " Tab View  q Quit  ";
        if (inspect_) shortcuts += "[/] Body  c Contacts";
        else {
            shortcuts += wide ? "a Restart with next algorithm" : "a Algorithm";
#if SALVAGE_ENABLE_THREADS
            shortcuts += wide ? "   w Change workers" : "  w Workers";
#endif
        }
        page.push_back(text(std::move(shortcuts)));
        if (!error_.empty()) page.push_back(paragraph(error_) | color(Color::RedLight));
        return vbox(std::move(page));
    }

    bool OnEvent(Event event) override {
        if (event == Event::Escape) app_.Exit();
        else if (event == Event::Tab || event == Event::TabReverse) inspect_ = !inspect_;
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
                case 'r': restart(simulation_.config().algorithm, simulation_.config().workers); break;
                case 'a':
                    restart(algorithms[(algorithmIndex() + 1) % algorithms.size()], simulation_.config().workers);
                    break;
#if SALVAGE_ENABLE_THREADS
                case 'w': {
                    const auto count = simulation_.config().workers;
                    const auto limit = std::max(1u, std::thread::hardware_concurrency());
                    restart(simulation_.config().algorithm, count >= limit ? 1 : std::min<std::size_t>(count * 2, limit));
                    break;
                }
#endif
                case '[': selectBody(false); break;
                case ']': selectBody(true); break;
                case 'c':
                    inspect_ = true;
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
        }
        if (playing_) app_.RequestAnimationFrame();
    }

private:
    std::size_t algorithmIndex() const {
        return static_cast<std::size_t>(std::ranges::find(algorithms, simulation_.config().algorithm) - algorithms.begin());
    }

    void restart(Algorithm algorithm, std::size_t workers) {
        playing_ = false;
        elapsed_ = 0;
        try {
            simulation_.restartWith(algorithm, workers);
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
        inspect_ = true;
        const auto count = simulation_.world().bodies.size();
        if (count == 0) return;
        if (forward) selected_ = (selected_ + 1) % count;
        else selected_ = selected_ ? selected_ - 1 : count - 1;
    }

    App& app_;
    core::Simulation simulation_;
    std::deque<core::Duration> history_;
    std::size_t selected_{};
    bool inspect_{}, showContacts_{true}, playing_{};
    double elapsed_{};
    std::string error_;
};

void runApp(core::SimulationConfig config, std::optional<SavedReport> initialReport) {
    if (!hasTerminal()) throw std::runtime_error("an interactive terminal is required");
    auto app = ftxui::App::Fullscreen();
    app.TrackMouse(false);
    auto host = Make<ComponentBase>();
    auto show = [&](const Component& page) {
        host->DetachAllChildren();
        host->Add(page);
    };
    std::shared_ptr<Dashboard> live;
    Component current;
    auto showLive = [&] {
        if (!live) live = Make<Dashboard>(app, std::move(config));
        current = live;
        show(current);
    };
    if (initialReport) {
        current = reportView(std::move(*initialReport));
        show(current);
    } else showLive();

    std::string path, error;
    InputOption options;
    options.multiline = false;
    auto input = Input(&path, "Path to a JSON or CSV report", options);
    auto prompt = Renderer(input, [&] {
        return vbox({text(" Open saved benchmark ") | bold, separator(), text(" Report file"),
                     input->Render() | border, paragraph(error) | color(Color::RedLight),
                     filler(), text(" Enter Open   Esc Cancel")});
    });
    bool opening = false;
    auto root = CatchEvent(host, [&](Event event) {
        if (opening) {
            if (event == Event::Return) {
                try {
                    current = reportView(loadReport(path));
                } catch (const std::exception& e) {
                    error = displayText(e.what());
                    return true;
                }
            } else if (event != Event::Escape) return false;
            opening = false;
            show(current);
            return true;
        }
        if (event == Event::o || event == Event::O) {
            if (live) live->pause();
            path.clear();
            error.clear();
            opening = true;
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
    app.PostEvent(ftxui::Event::Custom);
    app.Loop(root);
}
}

void runDashboard(core::SimulationConfig config) { runApp(std::move(config), {}); }

void runReport(const std::filesystem::path& path) { runApp({}, loadReport(path)); }

}
