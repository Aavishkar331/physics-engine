#ifndef PHYS_WORLD_H
#define PHYS_WORLD_H

#include "Body.h"

#include <cstdint>
#include <utility>
#include <vector>

namespace phys {

// Owns the bodies and advances the simulation. main.cpp only does input and
// rendering; all physics lives here.
class World {
public:
    Vector2 gravity{0.0f, 1000.0f};
    float fixedDt = 1.0f / 120.0f; // deterministic step
    int solverIterations = 8;      // relaxation passes per step
    bool useGrid = true;           // broad phase: grid vs naive O(n^2)

    int addBody(const Body &b);
    std::size_t size() const { return m_bodies.size(); }
    Body &at(std::size_t i) { return m_bodies[i]; }
    const Body &at(std::size_t i) const { return m_bodies[i]; }
    const std::vector<Body> &bodies() const { return m_bodies; }
    void clear() { m_bodies.clear(); m_accumulator = 0.0f; }

    // Index of the dynamic circle under `p`, or -1. Used for mouse picking.
    int queryPoint(Vector2 p) const;

    // Remove dynamic circles whose shape lies entirely outside [min, max]
    // (e.g. balls that fell off-screen). Static bodies are never removed.
    // Returns how many were discarded. Note: this invalidates body indices.
    int cullOutside(Vector2 min, Vector2 max);

    // Advance by a (possibly variable) frame time using a fixed-step
    // accumulator. Frame time is clamped to avoid the spiral of death.
    void step(float frameDt);

    // A single fixed step: integrate everything, then solve contacts.
    void stepOnce(float dt);

    // Candidate pairs for the narrow phase under the current broad phase.
    std::vector<std::pair<int, int>> broadPhasePairs() const;

    // Pairs that actually collide, via a given broad phase. For testing that
    // the grid and the naive path agree (sorted, deduplicated).
    std::vector<std::pair<int, int>> collidingPairs(bool grid) const;

private:
    std::vector<Body> m_bodies;
    float m_accumulator = 0.0f;

    std::vector<std::pair<int, int>> naivePairs() const;
    std::vector<std::pair<int, int>> gridPairs() const;
};

} // namespace phys

#endif // PHYS_WORLD_H
