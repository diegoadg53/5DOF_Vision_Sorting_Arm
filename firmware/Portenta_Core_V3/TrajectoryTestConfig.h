#pragma once
#include "ArmTrajectory.h"
#include "ClassificationConfig.h"
#include "PickMotionConfig.h"

// Only the imaginary part belongs to the test. Timing is identical to AUTO.
// Physical pickup settings live in PickMotionConfig.h and are shared with AUTO.
namespace TrajectoryTest {
using PickMotion::minimumXmm;
using PickMotion::maximumXmm;
using PickMotion::heightSlope;
using PickMotion::heightReferenceXmm;
using PickMotion::fittedContactZmm;
using PickMotion::pickupHeightAdjustmentMm;
using PickMotion::wristArrivalToleranceDeg;
using PickMotion::homeWaypointJointToleranceDeg;
using PickMotion::homeWaypointPositionToleranceMm;
using PickMotion::homeWaypointPositionHysteresisMm;
using PickMotion::surfaceTiltDegrees;
using PickMotion::beltContactZ;
constexpr float initialYmm = 400, velocityMmS = -100;

inline AutoConfig configuration(const AutoConfig &base, float xMm,
                              const RobotDHParams &params = robot_get_dh_params(),
                              uint8_t binId = 3) {
  AutoConfig result = base;
  Classification::configure(result, params);
  PickMotion::apply(result, xMm);
  // Bin 0 is a HOME/admission placeholder, not a routable destination.
  if (!binId)
    return result;
  bool selected = false;
  for (const Destination &destination : result.destinations)
    if (destination.binId == binId && destination.enabled) {
      memcpy(result.defaultDestination, destination.pose, sizeof(result.defaultDestination));
      selected = true;
    }
  if (!selected)
    result.defaultDestination[0] = NAN;
  return result;
}
} // namespace TrajectoryTest
