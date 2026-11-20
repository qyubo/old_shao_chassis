#pragma once

#include <cstdint>


namespace chassis {

// vx [m/s]: forward is positive.
// vy [m/s]: left is positive.
// vw [rad/s]: counter-clockwise viewed from above is positive.
struct velocity_command{
    float vx = 0.0f;  
    float vy = 0.0f;  
    float vw = 0.0f;   
};

enum class mode
{
    relax = 0,
    manual,
    patrol,
};

void run() noexcept;

}