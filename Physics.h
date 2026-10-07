#ifndef PHYSICS_H // Include Guard (Prevents double-import errors)
#define PHYSICS_H

#include "raylib.h"
#include <cmath>
#include <algorithm>
#include <vector>
using namespace std;

// --- MATH OPERATORS ---

inline Vector2 operator+(Vector2 a,Vector2 b)
{
    return {a.x+b.x,a.y+b.y};
}
inline Vector2 operator-(Vector2 a,Vector2 b)
{
    return {a.x-b.x,a.y-b.y};
}
inline float dot(Vector2 a,Vector2 b)
{
    return {a.x*b.x+a.y*b.y};
}
inline Vector2 operator*(Vector2 a,float b)
{
    return {a.x*b,a.y*b};
}
inline Vector2 operator*(float b,Vector2 a)
{
    return a*b;
}
inline float lensq(Vector2 a)
{
    return dot(a,a);
}
inline float len(Vector2 a)
{
    return sqrt(dot(a,a));
}
inline Vector2& operator+=(Vector2 &a, const Vector2 &b)
{
    a.x += b.x;
    a.y += b.y;
    return a;
}
inline Vector2& operator-=(Vector2 &a, const Vector2 &b)
{
    a.x -= b.x;
    a.y -= b.y;
    return a;
}

// --- PHYSICS STRUCTURES ---

enum ShapeType{CIRCLE,LINE_SEGMENT};

struct body 
{
    // 1. Shape Data
    ShapeType type;
    float radius;       // For Circles
    Vector2 start, end; // For Lines
    
    // 2. Physics Data
    Vector2 position;
    Vector2 velocity;
    Vector2 acceleration;
    float inverseMass;  // 0 = Static, >0 = Dynamic
    float restitution;  // Bounciness
    Color color;

    // --- FACTORY 1: Create Circle ---
    static body CreateCircle(Vector2 pos, Vector2 vel, float radius, float mass, Color color) 
    {
        body b;
        b.type = CIRCLE;
        b.position = pos;
        b.radius = radius;
        b.color = color;
        b.velocity = vel;
        b.acceleration = {0, 0};
        
        // Handle Mass
        if (mass > 0) b.inverseMass = 1.0f / mass;
        else b.inverseMass = 0.0f;
        
        b.restitution = 0.7f; 
        return b;
    }

    // --- FACTORY 2: Create Line (Wall) ---
    static body CreateLine(Vector2 start, Vector2 end, float res, Color color) 
    {
        body b;
        b.type = LINE_SEGMENT;
        b.start = start;
        b.end = end;
        b.color = color;
        b.position = {0, 0}; 
        b.velocity = {0, 0};
        b.acceleration = {0, 0};
        b.inverseMass = 0.0f; // Static!
        b.restitution = res; 
        return b;
    }

    // --- UPDATE ---
    void Update(float dt, Vector2 acceleration) 
    {
        if (inverseMass == 0.0f) return; // Don't move walls!
        position += velocity * dt + 0.5f * acceleration * dt * dt;
        velocity += acceleration * dt;
        acceleration = {0, 0};
    }
};

struct Manifold 
{
    bool isColliding;
    Vector2 normal; // Direction to push Body A away from Body B
    float depth;    // How much overlap exists
};

// 1. Circle vs Circle
Manifold CollideCircleCircle(const body& a, const body& b) 
{
    Manifold m = {false, {0,0}, 0.0f};
    
    Vector2 diff = b.position - a.position;
    float distSq = lensq(diff);
    float radiusSum = a.radius + b.radius;

    // Check if distance is less than sum of radii
    if (distSq < radiusSum * radiusSum) 
    {
        float dist = sqrt(distSq);
        
        // Safety: If balls are on exact same pixel, pick random direction
        if (dist == 0.0f) { dist = 1.0f; diff = {0, -1}; }
        
        m.isColliding = true;
        m.normal = diff * (1.0f / dist); // Normalized direction
        m.depth = radiusSum - dist;
    }
    return m;
}

// 2. Circle vs Line (The Wall Logic)
Manifold CollideCircleLine(const body& circle, const body& line) 
{
    Manifold m = {false, {0,0}, 0.0f};

    Vector2 lineVec = line.end - line.start;
    Vector2 ballToStart = circle.position - line.start;
    float lenSq = lensq(lineVec);
    
    // Project ball onto line (The 't' value logic)
    float t = dot(ballToStart, lineVec) / lenSq;
    t = max(0.0f, std::min(1.0f, t));

    Vector2 closestPoint = line.start + (lineVec * t);
    Vector2 distVec = closestPoint - circle.position; // Vector pointing TOWARDS line
    float distSq = lensq(distVec);

    if (distSq < circle.radius * circle.radius) 
    {
        float dist = sqrt(distSq);
        if (dist == 0)
        {
            float lineLen = sqrt(lenSq);
            m.normal = { -lineVec.y / lineLen, lineVec.x / lineLen }; // 90 degree rotation
            m.depth = circle.radius;
            m.isColliding = true;
            return m;
        }
        m.isColliding = true;
        // Important: Normal must point FROM line TO circle to push it out
        m.normal = distVec * (1.0f / dist); 
        m.depth = circle.radius - dist;
    }
    return m;
}

void ResolveCollision(body& a, body& b) 
{
    // 1. DETECT: Figure out who is hitting whom
    Manifold m = {false};

    if (a.type == CIRCLE && b.type == CIRCLE) 
    {
        m = CollideCircleCircle(a, b);
    } 
    else if (a.type == CIRCLE && b.type == LINE_SEGMENT) 
    {
        m = CollideCircleLine(a, b);
    }
    else if (a.type == LINE_SEGMENT && b.type == CIRCLE) 
    {
        // Swap inputs so the logic works (Line vs Circle -> Circle vs Line)
        m = CollideCircleLine(b, a);
        m.normal = m.normal * -1.0f; // Flip normal because we swapped!
    }

    // If no hit, we are done
    if (!m.isColliding) return;

    // 2. RESOLVE: Move them apart (Static Resolution)
    // If invMass is 0 (Static), it won't move.
    float totalInvMass = a.inverseMass + b.inverseMass;
    if (totalInvMass <= 0) return; // Two walls hitting? Ignore.

    Vector2 movePerIMass = m.normal * (m.depth / totalInvMass);
    a.position -= movePerIMass * a.inverseMass;
    b.position += movePerIMass * b.inverseMass;

    // 3. BOUNCE: Apply Velocity Impulse (Dynamic Resolution)
    Vector2 relativeVel = b.velocity - a.velocity;
    float velAlongNormal = dot(relativeVel, m.normal);

    // Only bounce if objects are moving TOWARDS each other
    if (velAlongNormal > 0) return;

    float e = min(a.restitution, b.restitution); // Use the lower bounciness
    // --- STABILITY FIX: Stop micro-bouncing ---
    // If the bounce velocity is tiny (like gravity pulling down), just kill it.
    if (std::abs(velAlongNormal) < 20.0f) { // Threshold depends on gravity
        e = 0.0f; // No bounce
    }
    float j = -(1.0f + e) * velAlongNormal;
    j /= totalInvMass;

    Vector2 impulse = m.normal * j;
    a.velocity -= impulse * a.inverseMass;
    b.velocity += impulse * b.inverseMass;
}

#endif