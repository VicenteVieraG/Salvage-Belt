#pragma once

#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>
#include "salvageCore.hpp"

namespace salvage::app {
    class Simulation {
        public:
            static constexpr double timeStep = 1.0 / 120.0;

            Simulation();

            const core::World& world() const noexcept { return this->scene; }
            const core::Body* selectedBody() const noexcept { return this->scene.getBody(this->selection); }
            std::span<const core::Contact> contacts() const noexcept { return this->currentContacts; }
            bool contactsVisible() const noexcept { return this->showContacts; }
            bool running() const noexcept { return this->playing; }
            std::uint64_t steps() const noexcept { return this->stepCount; }
            double simulationTime() const noexcept { return static_cast<double>(this->stepCount) * timeStep; }

            void toggleRunning() noexcept;
            void advance(double elapsed);
            void singleStep();
            void reset();
            void toggleContacts();

            void selectBody(core::BodyId id) noexcept;
            void cycleSelection(bool backwards) noexcept;

        private:
            void refreshContacts();

            core::World scene;
            core::BodyId selection{};
            std::vector<core::Contact> currentContacts;
            std::uint64_t stepCount{};
            double accumulatedTime{};
            bool playing{};
            bool showContacts{true};
    };

    std::string_view shapeName(const core::Body& body) noexcept;
    std::string formatVector(const core::Vec3& value);
}
