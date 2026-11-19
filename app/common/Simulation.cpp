#include "Simulation.hpp"

#include <algorithm>
#include <format>
#include <ranges>
#include <utility>

namespace salvage::app {
    Simulation::Simulation() {
        this->reset();
    }

    void Simulation::reset() {
        using namespace core;

        World demo{Algorithm::SweepAndPrune, {0.0, -9.81, 0.0}};
        demo.createBody(Body{{0.0, -0.4, 0.0}, {}, 0.0, BoxCollider{{9.0, 0.4, 9.0}}});
        const auto first = demo.createBody(
            Body{{-3.8, 3.2, 0.5}, {2.0, 0.0, 0.0}, 1.0, SphereCollider{0.95}, {0.65, 0.45}});
        demo.createBody(Body{{0.0, 4.0, 0.5}, {-1.1, 0.0, 0.0}, 1.0, BoxCollider{{0.8, 0.8, 0.8}}});
        demo.createBody(Body{{3.0, 6.0, -1.5}, {-0.4, 0.0, 0.0}, 0.8, SphereCollider{1.15}, {0.55, 0.5}});
        demo.createBody(Body{{-2.0, 0.6, -3.5}, {}, 1.0, BoxCollider{{1.1, 0.6, 0.7}}});
        demo.createBody(Body{{-2.0, 2.9, -3.5}, {}, 1.2, BoxCollider{{0.6, 0.6, 0.6}}});
        demo.createBody(Body{{2.1, 2.5, 3.9}, {-0.7, 0.0, -0.4}, 1.0, SphereCollider{0.7}, {0.5, 0.55}});

        this->scene = std::move(demo);
        this->selection = first;
        this->stepCount = 0;
        this->accumulatedTime = 0.0;
        this->playing = false;
        this->refreshContacts();
    }

    void Simulation::toggleRunning() noexcept {
        this->playing = !this->playing;
        this->accumulatedTime = 0.0;
    }

    void Simulation::advance(double elapsed) {
        if (!this->playing) return;

        // Limit catch-up after a suspend or a slow frame.
        this->accumulatedTime += std::clamp(elapsed, 0.0, 0.1);
        if (this->accumulatedTime < timeStep) return;

        while (this->accumulatedTime >= timeStep) {
            this->scene.step(timeStep);
            ++this->stepCount;
            this->accumulatedTime -= timeStep;
        }
        this->refreshContacts();
    }

    void Simulation::singleStep() {
        this->playing = false;
        this->accumulatedTime = 0.0;
        this->scene.step(timeStep);
        ++this->stepCount;
        this->refreshContacts();
    }

    void Simulation::toggleContacts() {
        this->showContacts = !this->showContacts;
        this->refreshContacts();
    }

    void Simulation::refreshContacts() {
        if (this->showContacts) {
            this->currentContacts = this->scene.findContacts();
        } else {
            this->currentContacts.clear();
        }
    }

    void Simulation::selectBody(core::BodyId id) noexcept {
        if (this->scene.getBody(id)) this->selection = id;
    }

    void Simulation::cycleSelection(bool backwards) noexcept {
        const auto bodies = this->scene.getBodies();
        if (bodies.empty()) return;

        const auto current = std::ranges::find(bodies, this->selection, &core::Body::getId);
        if (backwards) {
            this->selection = current == bodies.begin() ? bodies.back().getId() : (current - 1)->getId();
        } else {
            this->selection = current == bodies.end() || current + 1 == bodies.end()
                ? bodies.front().getId() : (current + 1)->getId();
        }
    }

    std::string_view shapeName(const core::Body& body) noexcept {
        return std::holds_alternative<core::SphereCollider>(body.getCollider()) ? "Sphere" : "Box";
    }

    std::string formatVector(const core::Vec3& value) {
        return std::format("({:.2f}, {:.2f}, {:.2f})", value.x, value.y, value.z);
    }
}
