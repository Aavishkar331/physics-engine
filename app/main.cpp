#include "raylib.h"

#include "Body.h"
#include "Vec2.h"
#include "World.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include <random>

using namespace phys;

// ---------------------------------------------------------------------------
// Scene helpers
// ---------------------------------------------------------------------------

static void addWalls(World &w) {
    w.addBody(Body::CreateLine({50, 750}, {750, 750}, 1.0f, GRAY)); // floor
    w.addBody(Body::CreateLine({50, 50}, {750, 50}, 1.0f, GRAY));   // ceiling
    w.addBody(Body::CreateLine({50, 50}, {50, 750}, 1.0f, GRAY));   // left
    w.addBody(Body::CreateLine({750, 50}, {750, 750}, 1.0f, GRAY)); // right
}

// Deterministic scene (same seed -> identical bodies), so the naive and grid
// paths are benchmarked and compared on exactly the same input.
static void buildScene(World &w, int n, unsigned seed) {
    w.clear();
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> rx(80.0f, 720.0f);
    std::uniform_real_distribution<float> ry(80.0f, 720.0f);
    std::uniform_real_distribution<float> rr(8.0f, 16.0f);
    std::uniform_real_distribution<float> rv(-150.0f, 150.0f);
    for (int i = 0; i < n; ++i) {
        w.addBody(Body::CreateCircle({rx(rng), ry(rng)}, {rv(rng), rv(rng)},
                                     rr(rng), 10.0f, BLUE));
    }
    addWalls(w);
}

// ---------------------------------------------------------------------------
// Benchmark mode (headless)
// ---------------------------------------------------------------------------

static bool selfTest() {
    World w;
    buildScene(w, 200, 12345u);
    for (int i = 0; i < 5; ++i) w.stepOnce(w.fixedDt); // create some overlaps
    return w.collidingPairs(false) == w.collidingPairs(true);
}

static double benchPath(int n, unsigned seed, bool grid, int steps) {
    World w;
    buildScene(w, n, seed);
    w.useGrid = grid;
    auto t0 = std::chrono::high_resolution_clock::now();
    for (int s = 0; s < steps; ++s) w.stepOnce(w.fixedDt);
    auto t1 = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return ms / steps;
}

static int runBench(int n, int steps) {
    // The naive path materialises every pair: n(n-1)/2 of them. Past this it
    // blows out memory/time with nothing to learn, so run grid only there.
    const int kNaiveMax = 10000;

    try {
        std::printf("pair-equivalence (naive vs grid): %s\n",
                    selfTest() ? "OK" : "FAIL");
        std::printf("N=%d, steps=%d\n", n, steps);
        std::printf("  budget: 16.6667 ms/step = 60 FPS\n");

        double grid = benchPath(n, 42u, true, steps);

        if (n <= kNaiveMax) {
            double naive = benchPath(n, 42u, false, steps);
            std::printf("  naive : %8.4f ms/step\n", naive);
            std::printf("  grid  : %8.4f ms/step  (%.2fx vs naive)\n", grid,
                        naive / grid);
        } else {
            std::printf("  naive : skipped (N > %d: O(n^2) pair list too large)\n",
                        kNaiveMax);
            std::printf("  grid  : %8.4f ms/step\n", grid);
        }
    } catch (const std::bad_alloc &) {
        std::fprintf(stderr,
                     "bench: out of memory at N=%d (too many bodies for this machine)\n",
                     n);
        return 1;
    }
    return selfTest() ? 0 : 1;
}

// ---------------------------------------------------------------------------
// Interactive mode
// ---------------------------------------------------------------------------

static int runInteractive() {
    const int screenWidth = 800;
    const int screenHeight = 800;

    InitWindow(screenWidth, screenHeight, "My Custom Physics Engine (CPU)");
    SetTargetFPS(60);

    World world;
    world.addBody(Body::CreateCircle({400, 100}, {50, 0}, 20.0f, 10.0f, MAROON));
    addWalls(world);

    int selected = -1;
    float savedInvMass = 0.0f;

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        // --- Input: toggle broad phase ---
        if (IsKeyPressed(KEY_B)) world.useGrid = !world.useGrid;

        // --- Input: spawn ---
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            Vector2 m = GetMousePosition();
            world.addBody(Body::CreateCircle(
                m, {(float)GetRandomValue(-200, 200), (float)GetRandomValue(-200, 0)},
                (float)GetRandomValue(10, 30), 10.0f, Fade(BLUE, 0.8f)));
        }

        // --- Input: grab ---
        if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
            selected = world.queryPoint(GetMousePosition());
            if (selected != -1) {
                savedInvMass = world.at(selected).inverseMass;
                world.at(selected).inverseMass = 0.0f; // freeze while held
            }
        }
        // --- Input: drag / throw ---
        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT) && selected != -1) {
            Vector2 m = GetMousePosition();
            Body &b = world.at(selected);
            if (dt > 0.0f) {
                Vector2 v = (m - b.position) * (1.0f / dt);
                float sp = len(v);
                const float kMaxThrow = 20000.0f; // keep sub-stepping bounded
                if (sp > kMaxThrow) v = v * (kMaxThrow / sp);
                b.velocity = v;
            }
            b.position = m;
        }
        if (IsMouseButtonReleased(MOUSE_BUTTON_RIGHT) && selected != -1) {
            world.at(selected).inverseMass = savedInvMass;
            selected = -1;
        }

        // --- Simulate ---
        world.step(dt);

        // Discard balls that have left the window (they'd be counted and
        // simulated forever otherwise). Skip while dragging: culling shifts
        // indices and would invalidate `selected`.
        if (selected == -1) {
            world.cullOutside({0, 0}, {(float)screenWidth, (float)screenHeight});
        }

        // --- Render ---
        BeginDrawing();
        ClearBackground(YELLOW);

        for (std::size_t i = 0; i < world.size(); ++i) {
            const Body &b = world.at(i);
            if (b.type == CIRCLE)
                DrawCircleV(b.position, b.radius, b.color);
            else
                DrawLineEx(b.start, b.end, 5.0f, b.color);
        }

        DrawText("L-Click: spawn   R-Click: grab/throw   B: broad phase", 10, 10,
                 20, BLACK);
        DrawText(TextFormat("Broad phase: %s", world.useGrid ? "GRID" : "NAIVE"),
                 10, 36, 20, world.useGrid ? DARKGREEN : MAROON);
        DrawText(TextFormat("Bodies: %d   FPS: %d", (int)world.size(), GetFPS()),
                 10, 62, 20, DARKGRAY);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}

int main(int argc, char **argv) {
    if (argc >= 2 && std::strcmp(argv[1], "--bench") == 0) {
        int n = (argc >= 3) ? std::atoi(argv[2]) : 1000;
        int steps = (argc >= 4) ? std::atoi(argv[3]) : 1000;
        if (n < 0) n = 0;
        if (steps < 1) steps = 1;
        return runBench(n, steps);
    }
    return runInteractive();
}
