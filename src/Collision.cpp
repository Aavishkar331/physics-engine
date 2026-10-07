#include "Collision.h"
#include "Vec2.h"

#include <algorithm>
#include <cmath>

namespace phys {

Manifold CollideCircleCircle(const Body &a, const Body &b) {
    Manifold m;

    Vector2 diff = b.position - a.position;
    float distSq = lensq(diff);
    float radiusSum = a.radius + b.radius;

    if (distSq < radiusSum * radiusSum) {
        float dist = std::sqrt(distSq);

        // Exactly coincident: pick an arbitrary separation axis.
        if (dist == 0.0f) {
            dist = 1.0f;
            diff = {0, -1};
        }

        m.isColliding = true;
        m.normal = diff * (1.0f / dist);
        m.depth = radiusSum - dist;
    }
    return m;
}

Manifold CollideCircleLine(const Body &circle, const Body &line) {
    Manifold m;

    Vector2 lineVec = line.end - line.start;
    Vector2 ballToStart = circle.position - line.start;
    float lenSq = lensq(lineVec);

    // Closest point on the segment to the circle centre (clamped projection).
    float t = (lenSq > 0.0f) ? dot(ballToStart, lineVec) / lenSq : 0.0f;
    t = std::max(0.0f, std::min(1.0f, t));

    Vector2 closest = line.start + lineVec * t;
    // Convention: the manifold normal points from A (the circle) towards B
    // (the line), i.e. the direction the circle must be pushed back along.
    Vector2 toLine = closest - circle.position; // circle -> line
    float distSq = lensq(toLine);

    if (distSq < circle.radius * circle.radius) {
        float dist = std::sqrt(distSq);
        if (dist == 0.0f) {
            // Centre lies on the line: push out perpendicular to it.
            float lineLen = std::sqrt(lenSq);
            if (lineLen > 0.0f) {
                m.normal = {-lineVec.y / lineLen, lineVec.x / lineLen};
            } else {
                m.normal = {0, -1};
            }
            m.depth = circle.radius;
            m.isColliding = true;
            return m;
        }
        m.isColliding = true;
        m.normal = toLine * (1.0f / dist); // circle -> line
        m.depth = circle.radius - dist;
    }
    return m;
}

bool detect(const Body &a, const Body &b, Manifold &m) {
    m = Manifold{};

    if (a.type == CIRCLE && b.type == CIRCLE) {
        m = CollideCircleCircle(a, b);
    } else if (a.type == CIRCLE && b.type == LINE_SEGMENT) {
        m = CollideCircleLine(a, b);
    } else if (a.type == LINE_SEGMENT && b.type == CIRCLE) {
        // Normal from CollideCircleLine points towards the circle (B).
        // Our convention is A -> B, so flip it.
        m = CollideCircleLine(b, a);
        m.normal = m.normal * -1.0f;
    }
    // LINE vs LINE: unsupported, stays non-colliding.
    return m.isColliding;
}

void resolve(Body &a, Body &b, const Manifold &m) {
    float totalInvMass = a.inverseMass + b.inverseMass;
    if (totalInvMass <= 0.0f) return; // two static bodies

    // Positional correction: separate the overlap, weighted by inverse mass.
    Vector2 movePerIMass = m.normal * (m.depth / totalInvMass);
    a.position -= movePerIMass * a.inverseMass;
    b.position += movePerIMass * b.inverseMass;

    // Impulse response along the contact normal.
    Vector2 relVel = b.velocity - a.velocity;
    float velAlongNormal = dot(relVel, m.normal);
    if (velAlongNormal > 0.0f) return; // already separating

    float e = std::min(a.restitution, b.restitution);
    // Kill micro-bounces so resting stacks settle instead of jittering.
    if (std::fabs(velAlongNormal) < 20.0f) e = 0.0f;

    float j = -(1.0f + e) * velAlongNormal / totalInvMass;
    Vector2 impulse = m.normal * j;
    a.velocity -= impulse * a.inverseMass;
    b.velocity += impulse * b.inverseMass;
}

void ResolveCollision(Body &a, Body &b) {
    Manifold m;
    if (detect(a, b, m)) resolve(a, b, m);
}

} // namespace phys
