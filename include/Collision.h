#ifndef PHYS_COLLISION_H
#define PHYS_COLLISION_H

#include "Body.h"

namespace phys {

struct Manifold {
    bool isColliding = false;
    Vector2 normal{0, 0}; // points from A towards B (direction to push B out)
    float depth = 0.0f;   // penetration depth
};

// Narrow-phase tests.
Manifold CollideCircleCircle(const Body &a, const Body &b);
Manifold CollideCircleLine(const Body &circle, const Body &line);

// Dispatch on shape types; fills `m` and returns whether A and B overlap.
bool detect(const Body &a, const Body &b, Manifold &m);

// Positional correction + impulse response for an already-detected contact.
void resolve(Body &a, Body &b, const Manifold &m);

// Convenience: detect then resolve. No-op when they don't touch.
void ResolveCollision(Body &a, Body &b);

} // namespace phys

#endif // PHYS_COLLISION_H
