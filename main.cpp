#include "raylib.h"
#include <algorithm>
#include <vector>
#include <string>
#include <cmath>
#include "Physics.h"
using namespace std;

int main() 
{
    // --- 2. Initialization ---
    const int screenWidth = 800;
    const int screenHeight = 800;
    
    InitWindow(screenWidth, screenHeight, "My Custom Physics Engine (CPU)");
    SetTargetFPS(60); // Essential for stable physics

    // Create a container for all our objects
    vector<body> bodies;


    // Let's spawn a ball
    bodies.push_back
    (
        body::CreateCircle
        (
            Vector2{400.0f, 100.0f}, // Position (Middle, Top)
            Vector2{50.0f, 0.0f},    // Velocity (Moving Right slightly)
            20.0f,              // Radius
            10.0f,
            MAROON
        )
    );
    // 1. The Floor
    bodies.push_back(body::CreateLine(Vector2{50, 750}, Vector2{750, 750},1, GRAY)); 
    // 2. The Ceiling
    bodies.push_back(body::CreateLine(Vector2{50, 50}, Vector2{750, 50},1, GRAY));
    // 3. Left Wall
    bodies.push_back(body::CreateLine(Vector2{50, 50}, Vector2{50, 750},1, GRAY));
    // 4. Right Wall
    bodies.push_back(body::CreateLine(Vector2{750, 50}, Vector2{750, 750},1, GRAY));

    // Physics Constants
    Vector2 gravity = { 0.0f, 1000.0f }; // Gravity points DOWN (positive Y)
    int selectedBody = -1; // -1 means "holding nothing"
    float originalInvMass = 0.0f;
    // --- 3. The Game Loop ---
    while (!WindowShouldClose()) 
    {
        
        // A. Calculate Time Step (dt)
        // This ensures your physics run at the same speed on fast and slow computers
        float dt = GetFrameTime(); 

        // B. Update Physics (The "Simulation" Step)
        for (auto &p : bodies) 
        {
            // 1. Apply Gravity
            // We multiply by dt here so gravity is applied "per second", not per frame
            p.Update(dt,{ gravity.x , gravity.y });

            for (size_t i = 0; i < bodies.size(); i++) 
            {
                for (size_t j = i + 1; j < bodies.size(); j++) 
                {
                    ResolveCollision(bodies[i], bodies[j]);
                }
            }
        }

        // C. Render (The "Drawing" Step)
        BeginDrawing();
            ClearBackground(YELLOW);

            // Draw Instructions
            DrawText("Left Click to Spawn Ball", 10, 10, 20, BLACK);

            // Inside main loop -> Render Step
            for (const auto &b :bodies) 
            {
                if(b.type == CIRCLE)
                {
                    DrawCircleV(b.position,b.radius,b.color);
                }
                else if (b.type == LINE_SEGMENT)
                {
                    DrawLineEx(b.start,b.end,5.0f,b.color);
                }
            }
        EndDrawing();

        // D. Input Handling (Spawn new balls)
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) 
        {
            Vector2 mousePos = GetMousePosition();
            bodies.push_back
            (
                body::CreateCircle
                (
                    Vector2{mousePos},
                    Vector2{(float)GetRandomValue(-200, 200),(float)GetRandomValue(-200, 0)},
                    (float)GetRandomValue(10, 30),
                    10.0f,
                    Fade(BLUE, 0.8f)
                )
            );
        }
        if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) 
        {
            Vector2 mousePos = GetMousePosition();
            // A. Find what we clicked on
            selectedBody = -1;
            for (size_t i = 0; i < bodies.size(); i++) 
            {
                body& b = bodies[i];
                
                // Ignore static walls (optional, remove if you want to drag walls!)
                if (b.inverseMass == 0) continue; 

                // Check Point vs Circle Collision
                Vector2 distVec = mousePos - b.position;
                if (lensq(distVec) < b.radius * b.radius) 
                {
                    selectedBody = i; // GRABBED IT!
                    originalInvMass = b.inverseMass; // 1. Save the normal mass
                    b.inverseMass = 0.0f;            // 2. Turn on "God Mode" (Static)
                    break; // Stop looking
                }
            }
        }

        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT) && selectedBody != -1) 
        {
            Vector2 mousePos = GetMousePosition();
            // B. Dragging (The "Throw" Logic)
            body& b = bodies[selectedBody];
            
            // TRICK: Calculate velocity based on how fast you are moving the mouse.
            // This makes the ball "fly" when you let go!
            // Velocity = Distance / Time
            b.velocity = (mousePos - b.position) * (1.0f / dt);
            
            // Snap position to mouse
            b.position = mousePos;
        }

        if (IsMouseButtonReleased(MOUSE_BUTTON_RIGHT)) 
        {
            if (selectedBody != -1) 
            {
                // Restore the mass so it becomes a normal physics object again
                bodies[selectedBody].inverseMass = originalInvMass;
            }
            // C. Release
            selectedBody = -1;
        }
    }
    CloseWindow();
    return 0;
}