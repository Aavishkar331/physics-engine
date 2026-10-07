#include "Body.h"
#include "Vec2.h"

#include <algorithm>

namespace phys {

Body Body::CreateCircle(Vector2 pos, Vector2 vel, float radius, float mass, Color color) {
    Body b;
    b.type = CIRCLE;
    b.position = pos;
    b.velocity = vel;
    b.acceleration = {0, 0};
    b.radius = radius;
    b.color = color;
    b.inverseMass = (mass > 0.0f) ? 1.0f / mass : 0.0f;
    b.restitution = 0.7f;
    return b;
}

Body Body::CreateLine(Vector2 start, Vector2 end, float restitution, Color color) {
    Body b;
    b.type = LINE_SEGMENT;
    b.start = start;
    b.end = end;
    b.color = color;
    b.position = {0, 0};
    b.velocity = {0, 0};
    b.acceleration = {0, 0};
    b.inverseMass = 0.0f; // walls never move
    b.restitution = restitution;
    return b;
}

void Body::integrate(float dt, Vector2 gravity) {
    if (isStatic()) return; // walls don't move

    // x += v*dt + 0.5*a*dt^2 ; v += a*dt
    position += velocity * dt + 0.5f * gravity * dt * dt;
    velocity += gravity * dt;
}

AABB Body::bounds() const {
    if (type == CIRCLE) {
        return { {position.x - radius, position.y - radius},
                 {position.x + radius, position.y + radius} };
    }
    // LINE_SEGMENT
    return { {std::min(start.x, end.x), std::min(start.y, end.y)},
             {std::max(start.x, end.x), std::max(start.y, end.y)} };
}

} // namespace phys
