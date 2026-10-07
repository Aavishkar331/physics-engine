#include "World.h"
#include "Collision.h"
#include "Vec2.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace phys {

// Upper bound on sub-steps per fixed step, so an absurd velocity can't make a
// single step take unbounded time.
static constexpr int kMaxSubSteps = 64;

int World::addBody(const Body &b) {
    m_bodies.push_back(b);
    return static_cast<int>(m_bodies.size()) - 1;
}

int World::queryPoint(Vector2 p) const {
    for (std::size_t i = 0; i < m_bodies.size(); ++i) {
        const Body &b = m_bodies[i];
        if (!b.isDynamicCircle()) continue;
        Vector2 d = p - b.position;
        if (lensq(d) < b.radius * b.radius) return static_cast<int>(i);
    }
    return -1;
}

int World::cullOutside(Vector2 min, Vector2 max) {
    int removed = 0;
    for (std::size_t i = m_bodies.size(); i-- > 0;) {
        const Body &b = m_bodies[i];
        if (!b.isDynamicCircle()) continue; // keep walls / static bodies
        if (b.position.x + b.radius < min.x || b.position.x - b.radius > max.x ||
            b.position.y + b.radius < min.y || b.position.y - b.radius > max.y) {
            m_bodies.erase(m_bodies.begin() + static_cast<std::ptrdiff_t>(i));
            ++removed;
        }
    }
    return removed;
}

void World::step(float frameDt) {
    // Clamp so a long stall can't queue a huge backlog of steps.
    if (frameDt > 0.25f) frameDt = 0.25f;

    m_accumulator += frameDt;
    while (m_accumulator >= fixedDt) {
        stepOnce(fixedDt);
        m_accumulator -= fixedDt;
    }
}

void World::stepOnce(float dt) {
    // Adaptive sub-stepping: cap each sub-step so no body travels more than half
    // the smallest circle radius, otherwise a fast ball tunnels straight through
    // a thin wall between two steps. Slow scenes resolve to a single sub-step.
    float maxSpeed = 0.0f;
    float minRadius = 0.0f;
    bool haveCircle = false;
    for (const Body &b : m_bodies) {
        if (b.inverseMass <= 0.0f) continue;
        float sp = len(b.velocity);
        if (sp > maxSpeed) maxSpeed = sp;
        if (b.type == CIRCLE && (!haveCircle || b.radius < minRadius)) {
            minRadius = b.radius;
            haveCircle = true;
        }
    }

    int sub = 1;
    if (haveCircle && maxSpeed > 0.0f) {
        float allowance = 0.5f * minRadius;
        if (allowance > 0.0f) {
            sub = static_cast<int>(std::ceil(maxSpeed * dt / allowance));
            sub = std::max(1, std::min(sub, kMaxSubSteps));
        }
    }

    const float h = dt / static_cast<float>(sub);
    for (int s = 0; s < sub; ++s) {
        for (Body &b : m_bodies) b.integrate(h, gravity);

        // Broad phase once per sub-step; narrow phase + resolution relaxed over
        // several iterations so stacked bodies settle without sinking.
        std::vector<std::pair<int, int>> pairs = broadPhasePairs();
        for (int it = 0; it < solverIterations; ++it) {
            for (const auto &pr : pairs) {
                ResolveCollision(m_bodies[pr.first], m_bodies[pr.second]);
            }
        }
    }
}

std::vector<std::pair<int, int>> World::broadPhasePairs() const {
    return useGrid ? gridPairs() : naivePairs();
}

std::vector<std::pair<int, int>> World::naivePairs() const {
    std::vector<std::pair<int, int>> pairs;
    const int n = static_cast<int>(m_bodies.size());
    pairs.reserve(static_cast<std::size_t>(n) * 2);
    for (int i = 0; i < n; ++i)
        for (int j = i + 1; j < n; ++j)
            pairs.emplace_back(i, j);
    return pairs;
}

std::vector<std::pair<int, int>> World::gridPairs() const {
    const int n = static_cast<int>(m_bodies.size());
    std::vector<std::pair<int, int>> pairs;
    if (n < 2) return pairs;

    // Cell size = 2 * largest circle radius. Any two overlapping AABBs then
    // share at least one cell, so inserting each body into every cell its AABB
    // touches and pairing within cells cannot miss a real contact.
    float maxRadius = 1.0f;
    for (const Body &b : m_bodies)
        if (b.type == CIRCLE) maxRadius = std::max(maxRadius, b.radius);
    const float cellSize = 2.0f * maxRadius;
    const float inv = 1.0f / cellSize;

    auto key = [](int cx, int cy) -> std::int64_t {
        return (static_cast<std::int64_t>(cx) << 32) ^
               (static_cast<std::uint32_t>(cy));
    };

    std::unordered_map<std::int64_t, std::vector<int>> grid;
    grid.reserve(static_cast<std::size_t>(n) * 2);

    for (int i = 0; i < n; ++i) {
        AABB box = m_bodies[i].bounds();
        int minx = static_cast<int>(std::floor(box.min.x * inv));
        int maxx = static_cast<int>(std::floor(box.max.x * inv));
        int miny = static_cast<int>(std::floor(box.min.y * inv));
        int maxy = static_cast<int>(std::floor(box.max.y * inv));
        for (int cx = minx; cx <= maxx; ++cx)
            for (int cy = miny; cy <= maxy; ++cy)
                grid[key(cx, cy)].push_back(i);
    }

    // Pair bodies sharing a cell.
    for (const auto &cell : grid) {
        const std::vector<int> &ids = cell.second;
        for (std::size_t a = 0; a < ids.size(); ++a)
            for (std::size_t b = a + 1; b < ids.size(); ++b) {
                int i = ids[a], j = ids[b];
                if (i > j) std::swap(i, j);
                pairs.emplace_back(i, j);
            }
    }

    // A pair can appear in several shared cells; deduplicate.
    std::sort(pairs.begin(), pairs.end());
    pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());
    return pairs;
}

std::vector<std::pair<int, int>> World::collidingPairs(bool grid) const {
    std::vector<std::pair<int, int>> candidates = grid ? gridPairs() : naivePairs();
    std::vector<std::pair<int, int>> hits;
    Manifold m;
    for (const auto &pr : candidates) {
        if (detect(m_bodies[pr.first], m_bodies[pr.second], m)) {
            hits.emplace_back(pr.first, pr.second);
        }
    }
    std::sort(hits.begin(), hits.end());
    hits.erase(std::unique(hits.begin(), hits.end()), hits.end());
    return hits;
}

} // namespace phys
