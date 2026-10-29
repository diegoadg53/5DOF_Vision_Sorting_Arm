#pragma once
// Build defaults. Runtime edits are stored separately; slope is not editable.
namespace PickMotion {
constexpr long minimumXmm = 200, maximumXmm = 450;
constexpr float heightSlope = 0.090678f;
constexpr float heightReferenceXmm = 300.0f, fittedContactZmm = -40.104f;
constexpr float pickupHeightAdjustmentMm = 0;
}
