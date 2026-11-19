#include "salvageAnalytic/Tui.hpp"
#include "Simulation.hpp"

#include <algorithm>
#include <chrono>
#include <format>
#include <iostream>
#include <string>
#include <thread>
#include <ftxui/component/app.hpp>
#include <ftxui/component/component.hpp>
#include <ftxui/component/event.hpp>
#include <ftxui/component/loop.hpp>
#include <ftxui/dom/elements.hpp>

#ifdef _WIN32
#include <io.h>
#else
#include <unistd.h>
#endif

namespace salvage::app {
    namespace {
        using namespace ftxui;

        bool hasTerminal() {
#ifdef _WIN32
            return _isatty(0) != 0 && _isatty(1) != 0;
#else
            return isatty(STDIN_FILENO) != 0 && isatty(STDOUT_FILENO) != 0;
#endif
        }

        Element bodyList(const Simulation& simulation) {
            Elements rows;
            const auto* selected = simulation.selectedBody();
            for (const auto& body : simulation.world().getBodies()) {
                const bool active = selected && selected->getId() == body.getId();
                const bool isStatic = body.getInverseMass() == 0.0;
                auto row = text(std::format("{} #{:<2} {}{}", active ? ">" : " ", body.getId(),
                                            shapeName(body), isStatic ? "  (static)" : ""));
                rows.push_back(active ? row | bold | color(Color::CyanLight) : row);
            }
            return window(text(" Bodies "), vbox(std::move(rows)));
        }

        Element bodyDetails(const core::Body* body) {
            if (!body) return text("No body selected");
            const auto& material = body->getMaterial();
            return vbox({
                text(std::format("Body #{}  {}", body->getId(), shapeName(*body))) | bold | color(Color::CyanLight),
                text("Position  " + formatVector(body->getPosition())),
                text("Velocity  " + formatVector(body->getVelocity())),
                text(std::format("Inverse mass  {:.2f}", body->getInverseMass())),
                text(std::format("Bounce {:.2f}   Friction {:.2f}", material.restitution, material.friction)),
            });
        }

        Element contactDetails(const Simulation& simulation) {
            if (!simulation.contactsVisible()) return text("Hidden");
            const auto contacts = simulation.contacts();
            if (contacts.empty()) return text("No contacts in the current frame");
            Elements rows{ text(std::format("{} touching {}", contacts.size(), contacts.size() == 1 ? "pair" : "pairs"))
                           | color(Color::YellowLight) };
            for (const auto& contact : contacts.first(std::min<std::size_t>(contacts.size(), 3))) {
                rows.push_back(text(std::format("#{} / #{}    Depth {:.5f}",
                                                contact.first, contact.second, contact.penetration)));
            }
            if (contacts.size() > 3) rows.push_back(text(std::format("+{} more", contacts.size() - 3)));
            return vbox(std::move(rows));
        }

        struct ControlLabels {
            std::string play = "Play";
            std::string contacts = "Contacts on";
        };

        Component playbackControls(Simulation& simulation, App& screen, ControlLabels& labels) {
            const auto style = ButtonOption::Ascii();
            return Container::Vertical({
                Container::Horizontal({
                    Button(&labels.play, [&] { simulation.toggleRunning(); }, style),
                    Button("Step", [&] { simulation.singleStep(); }, style),
                    Button("Reset", [&] { simulation.reset(); }, style),
                    Button(&labels.contacts, [&] { simulation.toggleContacts(); }, style),
                }),
                Container::Horizontal({
                    Button("Previous body", [&] { simulation.cycleSelection(true); }, style),
                    Button("Next body", [&] { simulation.cycleSelection(false); }, style),
                    Button("Quit", screen.ExitLoopClosure(), style),
                }),
            });
        }

        bool handleShortcut(const Event& event, Simulation& simulation, App& screen) {
            if (event == Event::Character(' ')) simulation.toggleRunning();
            else if (event == Event::Character('n')) simulation.singleStep();
            else if (event == Event::Character('r')) simulation.reset();
            else if (event == Event::Character('c')) simulation.toggleContacts();
            else if (event == Event::Character('[')) simulation.cycleSelection(true);
            else if (event == Event::Character(']')) simulation.cycleSelection(false);
            else if (event == Event::Character('q') || event == Event::Escape) screen.Exit();
            else return false;
            return true;
        }
    }

    int runTui() {
        using namespace std::chrono;
        if (!hasTerminal()) {
            std::cerr << "salvageAnalytic requires an interactive terminal.\n";
            return 1;
        }
        Simulation simulation;
        auto screen = ftxui::App::Fullscreen();
        ControlLabels labels;
        auto controls = playbackControls(simulation, screen, labels);
        auto view = Renderer(controls, [&] {
            labels.play = simulation.running() ? "Pause" : "Play";
            labels.contacts = simulation.contactsVisible() ? "Contacts on" : "Contacts off";
            const auto status = std::format("{}   Step {}   {:.2f} s",
                simulation.running() ? "Running" : "Paused", simulation.steps(), simulation.simulationTime());
            auto details = window(text(" Inspector "), vbox({
                bodyDetails(simulation.selectedBody()),
                separator(),
                text("Current contacts") | bold,
                contactDetails(simulation),
            }));
            auto scene = screen.dimx() >= 68
                ? hbox({bodyList(simulation) | size(WIDTH, EQUAL, 22), details | flex})
                : details;
            return vbox({
                text(" SALVAGE ANALYTIC ") | bold | color(Color::CyanLight),
                text(status),
                separator(),
                controls->Render(),
                scene,
                filler(),
                text("Space  Play/Pause   N  Step   R  Reset") | dim,
                text("C  Contacts   [ / ]  Select body   Q  Quit") | dim,
            });
        });
        view = CatchEvent(view, [&](const Event& event) { return handleShortcut(event, simulation, screen); });

        Loop loop(&screen, view);
        screen.PostEvent(Event::Custom);
        auto previous = steady_clock::now();
        while (!loop.HasQuitted()) {
            const auto now = steady_clock::now();
            if (simulation.running()) {
                simulation.advance(duration<double>(now - previous).count());
                screen.PostEvent(Event::Custom);
            }
            previous = now;
            loop.RunOnce();
            if (!loop.HasQuitted()) std::this_thread::sleep_until(now + milliseconds(33));
        }
        return 0;
    }
}
