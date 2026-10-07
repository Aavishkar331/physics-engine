#ifndef PHYS_BODY_H
#define PHYS_BODY_H

#include "raylib.h"

namespace phys {

enum ShapeType { CIRCLE, LINE_SEGMENT };

// Axis-aligned bounding box, used by the broad phase.
struct AABB {
    Vector2 min;
    Vector2 max;
};

struct Body {
    // --- Shape ---
    ShapeType type = CIRCLE;
    float radius = 0.0f;        // circles
    Vector2 start{0, 0};        // line segments
    Vector2 end{0, 0};

    // --- Dynamics ---
    Vector2 position{0, 0};
    Vector2 velocity{0, 0};
    Vector2 acceleration{0, 0};
    float inverseMass = 0.0f;   // 0 = static (infinite mass)
    float restitution = 0.7f;   // bounciness

    Color color{255, 255, 255, 255};

    // Factories
    static Body CreateCircle(Vector2 pos, Vector2 vel, float radius, float mass, Color color);
    static Body CreateLine(Vector2 start, Vector2 end, float restitution, Color color);

    // Semi-implicit Euler integration. `gravity` is an acceleration (units/s^2).
    void integrate(float dt, Vector2 gravity);

    bool isStatic() const { return inverseMass == 0.0f; }
    bool isDynamicCircle() const { return type == CIRCLE && inverseMass > 0.0f; }

    // Conservative bounds of the shape, for broad-phase bucketing.
    AABB bounds() const;
};

} // namespace phys

#endif // PHYS_BODY_H
