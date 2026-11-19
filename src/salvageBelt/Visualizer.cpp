#include "salvageBelt/Visualizer.hpp"
#include "Simulation.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <raylib.h>
#include <rlgl.h>
#include <raygui.h>

namespace salvage::app {
    namespace {
        constexpr Color background{15, 21, 31, 255};
        constexpr Color panel{23, 32, 46, 255};
        constexpr Color rule{45, 60, 78, 255};
        constexpr Color ink{234, 241, 249, 255};
        constexpr Color quiet{158, 177, 198, 255};
        constexpr Color cyan{112, 222, 223, 255};
        constexpr Color amber{255, 190, 104, 255};
        constexpr int headerHeight = 68;
        constexpr int footerHeight = 44;
        constexpr int sidebarWidth = 306;

        struct Window {
            ~Window() { CloseWindow(); }
        };

        class Viewport {
            public:
                Viewport() = default;
                Viewport(const Viewport&) = delete;
                Viewport& operator=(const Viewport&) = delete;
                ~Viewport() { if (this->target.id) UnloadRenderTexture(this->target); }

                RenderTexture2D fit(int width, int height) {
                    if (this->target.texture.width == width && this->target.texture.height == height) {
                        return this->target;
                    }
                    if (this->target.id) UnloadRenderTexture(this->target);
                    this->target = LoadRenderTexture(width, height);
                    if (!IsRenderTextureValid(this->target)) {
                        throw std::runtime_error("Could not create the 3D viewport.");
                    }
                    return this->target;
                }

            private:
                RenderTexture2D target{};
        };

        struct Orbit {
            float azimuth{0.67f};
            float elevation{0.63f};
            float distance{29.5f};

            [[nodiscard]] Camera3D camera() const {
                const Vector3 target{0.0f, 1.2f, 0.0f};
                const float levelRadius = this->distance * std::cos(this->elevation);
                const Vector3 position{
                    target.x + levelRadius * std::sin(this->azimuth),
                    target.y + this->distance * std::sin(this->elevation),
                    target.z + levelRadius * std::cos(this->azimuth),
                };
                return {position, target, {0.0f, 1.0f, 0.0f}, 48.0f, CAMERA_PERSPECTIVE};
            }

            void update() {
                if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
                    const auto drag = GetMouseDelta();
                    this->azimuth = std::remainder(this->azimuth - drag.x * 0.008f, 2.0f * std::numbers::pi_v<float>);
                    this->elevation = std::clamp(this->elevation + drag.y * 0.008f, 0.12f, 1.45f);
                }
                this->distance = std::clamp(this->distance * std::exp(-0.10f * GetMouseWheelMove()), 6.0f, 55.0f);
            }
        };

        Vector3 toRay(const core::Vec3& value) {
            return {static_cast<float>(value.x), static_cast<float>(value.y), static_cast<float>(value.z)};
        }

        BoundingBox toRay(const core::AABB& bounds) {
            return {toRay(bounds.min), toRay(bounds.max)};
        }

        void pickBody(Simulation& simulation, Ray ray) {
            float nearest = std::numeric_limits<float>::infinity();
            for (const auto& body : simulation.world().getBodies()) {
                const auto* sphere = std::get_if<core::SphereCollider>(&body.getCollider());
                const auto hit = sphere
                    ? GetRayCollisionSphere(ray, toRay(body.getPosition()), static_cast<float>(sphere->radius))
                    : GetRayCollisionBox(ray, toRay(body.aabb()));
                if (hit.hit && hit.distance < nearest) {
                    nearest = hit.distance;
                    simulation.selectBody(body.getId());
                }
            }
        }

        void drawBody(const core::Body& body, bool selected) {
            const auto position = toRay(body.getPosition());
            const auto* sphere = std::get_if<core::SphereCollider>(&body.getCollider());
            const auto wire = selected ? ink : Color{29, 51, 69, 255};
            if (sphere) {
                const auto radius = static_cast<float>(sphere->radius);
                DrawSphereEx(position, radius, 18, 30, selected ? cyan : Color{92, 158, 189, 255});
                const auto outline = radius * 1.008f;
                DrawCircle3D(position, outline, {1, 0, 0}, 90, wire);
                DrawCircle3D(position, outline, {0, 1, 0}, 90, wire);
                DrawCircle3D(position, outline, {0, 0, 1}, 0, wire);
            } else {
                const auto size = toRay(body.aabb().max - body.aabb().min);
                const auto fill = body.getInverseMass() == 0.0
                    ? Color{32, 46, 63, 255} : (selected ? cyan : Color{139, 141, 202, 255});
                DrawCubeV(position, size, fill);
                DrawCubeWiresV(position, {size.x + 0.01f, size.y + 0.01f, size.z + 0.01f}, wire);
            }
        }

        void drawFloorGrid() {
            for (int line = -9; line <= 9; ++line) {
                constexpr float y = 0.008f;
                const float p = static_cast<float>(line);
                DrawLine3D({p, y, -9.0f}, {p, y, 9.0f}, rule);
                DrawLine3D({-9.0f, y, p}, {9.0f, y, p}, rule);
            }
        }

        void renderScene(RenderTexture2D target, Camera3D camera, const Simulation& simulation) {
            BeginTextureMode(target);
            ClearBackground(background);
            BeginMode3D(camera);

            const auto* selected = simulation.selectedBody();
            for (const auto& body : simulation.world().getBodies()) {
                drawBody(body, &body == selected);
                if (body.getInverseMass() == 0.0) drawFloorGrid();
            }

            // Draw markers through body surfaces so a contact remains visible when it is occluded.
            if (!simulation.contacts().empty()) {
                rlDrawRenderBatchActive();
                rlDisableDepthTest();
                for (const auto& contact : simulation.contacts()) {
                    DrawSphereEx(toRay(contact.point), 0.085f, 6, 8, amber);
                    DrawLine3D(toRay(contact.point), toRay(contact.point + contact.normal * 0.65), amber);
                }
                rlDrawRenderBatchActive();
                rlEnableDepthTest();
            }
            EndMode3D();
            EndTextureMode();
        }

        void configureButtons() {
            GuiLoadStyleDefault();
            GuiSetStyle(DEFAULT, TEXT_SIZE, 16);
            GuiSetStyle(BUTTON, BORDER_WIDTH, 1);
            GuiSetStyle(BUTTON, BORDER_COLOR_NORMAL, ColorToInt(rule));
            GuiSetStyle(BUTTON, BASE_COLOR_NORMAL, ColorToInt(background));
            GuiSetStyle(BUTTON, TEXT_COLOR_NORMAL, ColorToInt(ink));
            GuiSetStyle(BUTTON, BORDER_COLOR_FOCUSED, ColorToInt(cyan));
            GuiSetStyle(BUTTON, BASE_COLOR_FOCUSED, ColorToInt(panel));
            GuiSetStyle(BUTTON, TEXT_COLOR_FOCUSED, ColorToInt(cyan));
            GuiSetStyle(BUTTON, BORDER_COLOR_PRESSED, ColorToInt(cyan));
            GuiSetStyle(BUTTON, BASE_COLOR_PRESSED, ColorToInt(cyan));
            GuiSetStyle(BUTTON, TEXT_COLOR_PRESSED, ColorToInt(background));
        }

        void drawDetails(const core::Body& body, int x, int y) {
            const auto title = std::format("#{}  {}", body.getId(), shapeName(body));
            DrawText(title.c_str(), x, y, 28, cyan);
            DrawText(body.getInverseMass() == 0.0 ? "Static" : "Dynamic", x, y + 37, 16, quiet);

            DrawText("Position", x, y + 72, 16, quiet);
            DrawText(formatVector(body.getPosition()).c_str(), x, y + 94, 20, ink);
            DrawText("Velocity", x, y + 130, 16, quiet);
            DrawText(formatVector(body.getVelocity()).c_str(), x, y + 152, 20, ink);

            const auto& material = body.getMaterial();
            DrawText(std::format("Inverse mass    {:.2f}", body.getInverseMass()).c_str(), x, y + 198, 18, ink);
            DrawText(std::format("Restitution        {:.2f}", material.restitution).c_str(), x, y + 225, 18, ink);
            DrawText(std::format("Friction              {:.2f}", material.friction).c_str(), x, y + 252, 18, ink);
        }

        void drawSidebar(Simulation& simulation, int left, int height) {
            DrawRectangle(left, headerHeight, sidebarWidth, height, panel);
            DrawLine(left, headerHeight, left, headerHeight + height, rule);
            const int x = left + 22;
            const int top = headerHeight + 21;
            const auto count = std::format("BODY INSPECTOR   /   {} BODIES", simulation.world().getBodyCount());
            DrawText(count.c_str(), x, top, 15, quiet);
            if (const auto* body = simulation.selectedBody()) drawDetails(*body, x, top + 29);

            if (GuiButton({static_cast<float>(x), static_cast<float>(top + 315), 124, 34}, "Previous body")) {
                simulation.cycleSelection(true);
            }
            if (GuiButton({static_cast<float>(x + 138), static_cast<float>(top + 315), 124, 34}, "Next body")) {
                simulation.cycleSelection(false);
            }

            DrawLine(x, top + 370, x + 262, top + 370, rule);
            DrawText("CURRENT CONTACTS", x, top + 391, 15, quiet);
            const auto contactCount = simulation.contacts().size();
            const auto contacts = simulation.contactsVisible()
                ? std::format("{} touching {}", contactCount, contactCount == 1 ? "pair" : "pairs") : "Hidden";
            DrawText(contacts.c_str(), x, top + 415, 20, simulation.contactsVisible() ? amber : quiet);
            if (simulation.contactsVisible()) DrawText("Markers show point and normal", x, top + 443, 15, quiet);
        }

        void drawHeader(Simulation& simulation) {
            DrawRectangle(0, 0, GetScreenWidth(), headerHeight, panel);
            DrawLine(0, headerHeight - 1, GetScreenWidth(), headerHeight - 1, rule);
            DrawText("SALVAGE BELT", 22, 23, 24, ink);

            if (GuiButton({241, 16, 86, 36}, simulation.running() ? "Pause" : "Play")) simulation.toggleRunning();
            if (GuiButton({335, 16, 70, 36}, "Step")) simulation.singleStep();
            if (GuiButton({413, 16, 70, 36}, "Reset")) simulation.reset();
            if (GuiButton({491, 16, 122, 36}, simulation.contactsVisible() ? "Contacts on" : "Contacts off")) {
                simulation.toggleContacts();
            }

            const auto status = std::format("{}   |   {:.2f} s   /   Step {}",
                simulation.running() ? "Running" : "Paused", simulation.simulationTime(), simulation.steps());
            DrawText(status.c_str(), GetScreenWidth() - MeasureText(status.c_str(), 18) - 22, 25, 18, quiet);
        }

        void handleShortcuts(Simulation& simulation) {
            if (IsKeyPressed(KEY_SPACE)) simulation.toggleRunning();
            if (IsKeyPressed(KEY_N)) simulation.singleStep();
            if (IsKeyPressed(KEY_R)) simulation.reset();
            if (IsKeyPressed(KEY_C)) simulation.toggleContacts();
            if (IsKeyPressed(KEY_LEFT_BRACKET)) simulation.cycleSelection(true);
            if (IsKeyPressed(KEY_RIGHT_BRACKET)) simulation.cycleSelection(false);
        }
    }

    int runVisualizer() {
        Simulation simulation;
        SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_VSYNC_HINT);
        InitWindow(1120, 720, "Salvage Belt | Visualizer");
        if (!IsWindowReady()) throw std::runtime_error("Could not open a graphical window.");
        const Window window;
        SetWindowMinSize(980, 610);
        SetTargetFPS(60);
        configureButtons();

        Viewport viewport;
        Orbit orbit;
        while (!WindowShouldClose()) {
            simulation.advance(static_cast<double>(GetFrameTime()));
            handleShortcuts(simulation);
            const int width = GetScreenWidth() - sidebarWidth;
            const int height = GetScreenHeight() - headerHeight - footerHeight;
            if (width <= 0 || height <= 0) {
                PollInputEvents();
                continue;
            }
            const auto mouse = GetMousePosition();
            const Vector2 localMouse{mouse.x, mouse.y - static_cast<float>(headerHeight)};
            const bool overScene = CheckCollisionPointRec(localMouse, {0, 0, static_cast<float>(width),
                                                                        static_cast<float>(height)});
            if (overScene) orbit.update();

            const auto camera = orbit.camera();
            if (overScene && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                pickBody(simulation, GetScreenToWorldRayEx(localMouse, camera, width, height));
            }
            const auto target = viewport.fit(width, height);
            renderScene(target, camera, simulation);

            BeginDrawing();
            ClearBackground(background);
            DrawTextureRec(target.texture, {0, 0, static_cast<float>(width), -static_cast<float>(height)},
                           {0, headerHeight}, WHITE);
            drawSidebar(simulation, width, height);
            drawHeader(simulation);
            const int footer = GetScreenHeight() - footerHeight;
            DrawLine(0, footer, GetScreenWidth(), footer, rule);
            DrawText("Left click  Select    Right drag  Orbit    Scroll  Zoom", 22, footer + 14, 16, quiet);
            const char* shortcuts = "Space  Play/Pause   N  Step   R  Reset   C  Contacts   Esc  Quit";
            DrawText(shortcuts, GetScreenWidth() - MeasureText(shortcuts, 16) - 22, footer + 14, 16, quiet);
            EndDrawing();
        }
        return 0;
    }
}
