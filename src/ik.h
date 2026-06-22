#pragma once
#include <Arduino.h>

// 5-bar linkage dimensions (mm)
static constexpr float IK_L1 = 60.0f;
static constexpr float IK_L2 = 60.0f;
static constexpr float IK_L3 = 60.0f;
static constexpr float IK_L4 = 60.0f;
static constexpr float IK_D  = 36.0f;

// Elbow-out IK for symmetric 5-bar linkage (pivots at (0,0) and (D,0)).
// Returns joint angles t1 (left pivot) and t2 (right pivot) in radians.
// Returns false if (x,y) is outside reachable workspace.
inline bool calcIK(float x, float y, float& t1, float& t2) {
    float d1sq = x*x + y*y;
    float d1   = sqrtf(d1sq);
    if (d1 > IK_L1+IK_L3 || d1 < fabsf(IK_L1-IK_L3)+0.01f) return false;
    float cosB1 = constrain((IK_L1*IK_L1+d1sq-IK_L3*IK_L3)/(2.0f*IK_L1*d1), -1.0f, 1.0f);
    t1 = atan2f(y, x) + acosf(cosB1);

    float dx2 = x - IK_D, dy2 = y;
    float d2sq = dx2*dx2 + dy2*dy2, d2 = sqrtf(d2sq);
    if (d2 > IK_L2+IK_L4 || d2 < fabsf(IK_L2-IK_L4)+0.01f) return false;
    float cosB2 = constrain((IK_L2*IK_L2+d2sq-IK_L4*IK_L4)/(2.0f*IK_L2*d2), -1.0f, 1.0f);
    t2 = atan2f(dy2, dx2) - acosf(cosB2);
    return true;
}
