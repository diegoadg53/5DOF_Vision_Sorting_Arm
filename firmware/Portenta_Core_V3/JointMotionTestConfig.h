#pragma once
#include <stdint.h>
namespace JointMotionTestSettings {
// Reuse validated pickup/sorting geometry; keep the load path repeatable.
constexpr unsigned cycles = 50;
constexpr float pickupXmm = 332;
constexpr uint8_t binId = 3;
constexpr uint32_t baselineDwellMs = 5000, homeDwellMs = 5000;
// These stop a large *reported* drift during a stationary HOME dwell.
// They do not compensate offsets or replace the level as an external reference.
constexpr float maximumHomeJointDriftDeg = 3;
}
