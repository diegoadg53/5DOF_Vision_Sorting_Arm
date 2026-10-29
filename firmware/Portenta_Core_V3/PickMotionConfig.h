#pragma once
#include "ArmTrajectory.h"

// Shared physical settings for camera AUTO and the independent trajectory test.
// Edit pickup geometry and arrival allowances here so both modes stay aligned.
namespace PickMotion {
constexpr float wristArrivalToleranceDeg = 1.5f;
constexpr float wristHoldingToleranceDeg = 2.0f;
constexpr float homeWaypointJointToleranceDeg[5] = {1, 1, 1, wristArrivalToleranceDeg, 1};
constexpr float homeWaypointPositionToleranceMm = 5;
constexpr float homeWaypointPositionHysteresisMm = 1;
// WIDE and TALL: transport starts this long after closingDurationMs elapses,
// measured from the actual close command. This is a code setting, not Flash tuning.
constexpr uint32_t pickupPostCloseMs = 100;

inline bool supportsX(float xMm) {
  return isfinite(xMm) && xMm >= minimumXmm && xMm <= maximumXmm;
}
inline float surfaceTiltDegrees() {
  return atanf(heightSlope) * 180.0f / PI;
}
inline float beltContactZ(float xMm) {
  return fittedContactZmm + pickupHeightAdjustmentMm + heightSlope * (xMm - heightReferenceXmm);
}
inline float beltContactZ(float xMm, const AutoConfig &config) {
  return config.fittedContactZmm + config.pickupHeightAdjustmentMm +
         heightSlope * (xMm - config.heightReferenceXmm);
}
inline void apply(AutoConfig &config, float xMm) {
  // Never extrapolate the physically tested belt fit to a new pickup lane.
  config.pickupZ = supportsX(xMm) ? beltContactZ(xMm, config) : NAN;
  config.pickupRoll = -surfaceTiltDegrees();
  config.pickupPitch = 0;
  config.pickupYaw = 90;
  config.boxOrientation = true;
  config.pickupTimedCompletion = true;
  config.pickupPostCloseMs = pickupPostCloseMs;
  // Keep tilt toward the same base-X direction at either jaw heading (ZYX).
  config.approachRoll = 0;
  config.approachPitch = -surfaceTiltDegrees();
  config.approachYaw = 0;
  config.pickupTask = IkTask::FullPose;
  // Sorting joint locks belong to Classification::configure(), not pickup geometry.
  config.jointToleranceOverrideDeg[3] = wristArrivalToleranceDeg;
  config.jointHoldHysteresisDeg[3] = wristHoldingToleranceDeg - wristArrivalToleranceDeg;
  config.useHomeWaypointArrival = true;
  for (int i = 0; i < 5; ++i)
    config.homeWaypointArrival.jointToleranceDeg[i] = homeWaypointJointToleranceDeg[i];
  config.homeWaypointArrival.positionToleranceMm = homeWaypointPositionToleranceMm;
  config.homeWaypointArrival.positionHysteresisMm = homeWaypointPositionHysteresisMm;
}
} // namespace PickMotion
