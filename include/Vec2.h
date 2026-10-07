#ifndef PHYS_VEC2_H
#define PHYS_VEC2_H

// 2D vector math built on top of Raylib's Vector2.
// Header-only: every function is `inline`, so including this in several
// translation units does not violate the One Definition Rule.

#include "raylib.h"
#include <cmath>

namespace phys {

inline Vector2 operator+(Vector2 a, Vector2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vector2 operator-(Vector2 a, Vector2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vector2 operator*(Vector2 a, float s)   { return {a.x * s, a.y * s}; }
inline Vector2 operator*(float s, Vector2 a)   { return a * s; }

inline Vector2 &operator+=(Vector2 &a, Vector2 b) { a.x += b.x; a.y += b.y; return a; }
inline Vector2 &operator-=(Vector2 &a, Vector2 b) { a.x -= b.x; a.y -= b.y; return a; }

inline float dot(Vector2 a, Vector2 b) { return a.x * b.x + a.y * b.y; }
inline float lensq(Vector2 a)          { return dot(a, a); }
inline float len(Vector2 a)            { return std::sqrt(dot(a, a)); }

// Unit vector; returns {0,0} for a zero-length input.
inline Vector2 normalized(Vector2 a) {
    float l = len(a);
    return (l > 0.0f) ? a * (1.0f / l) : Vector2{0.0f, 0.0f};
}

} // namespace phys

#endif // PHYS_VEC2_H
