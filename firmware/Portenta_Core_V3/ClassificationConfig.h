#pragma once
#include "ArmTrajectory.h"
#include <string.h>

// Shared by AUTO's ID-aware routing and the camera-independent trajectory test.
namespace Classification {
constexpr size_t binCount = 3;
static_assert(sizeof(AutoConfig().destinations) / sizeof(Destination) == binCount,
              "Bin definitions must match AutoConfig destinations");
// Model reference TCP. It is the far end of the roll sweep (bin 3).
// With shoulder pitch=0 and reference shoulder roll=90deg, +14.545deg TCP
// pitch reaches a 300mm radius at Z=170mm (within the IK planning tolerance).
constexpr float referencePose[6] = {-76, 300, 130, 0, 9.583f, 90};
struct BinDefinition {
  uint8_t binId;
  float rollOffsetDegrees;
  bool enabled;
};
constexpr BinDefinition bins[binCount] = {{1, -40, true}, {2, -20, true}, {3, 0, true}};

struct PartRoute {
  uint8_t partId, binId; // binId=0 explicitly ignores this ID.
};
// Edit/append rows here. Several part IDs can share a bin. An omitted ID is
// ignored as well; duplicate part IDs are configuration errors.
constexpr PartRoute partRoutes[] = {{2, 1}, {6, 2}, {1, 3}};
constexpr size_t routeCount = sizeof(partRoutes) / sizeof(partRoutes[0]);

inline bool mappingValid(const PartRoute *routes = partRoutes, size_t count = routeCount) {
  for (size_t i = 0; i < count; ++i) {
    if (routes[i].binId > binCount)
      return false;
    for (size_t j = 0; j < i; ++j)
      if (routes[i].partId == routes[j].partId)
        return false;
  }
  return true;
}
inline uint8_t binForPartId(uint8_t id, const PartRoute *routes = partRoutes,
                          size_t count = routeCount) {
  for (size_t i = 0; i < count; ++i)
    if (routes[i].partId == id)
      return routes[i].binId <= binCount ? routes[i].binId : 0;
  return 0;
}
inline const Destination *destinationForPart(const AutoConfig &config, uint8_t id) {
  uint8_t binId = binForPartId(id);
  if (!binId)
    return nullptr;
  for (const Destination &destination : config.destinations)
    if (destination.binId == binId && destination.enabled)
      return &destination;
  return nullptr;
}
inline size_t activeRouteCount(const AutoConfig &config) {
  if (!mappingValid())
    return 0;
  size_t count = 0;
  for (const PartRoute &route : partRoutes)
    if (destinationForPart(config, route.partId))
      ++count;
  return count;
}
inline const PartRoute *activeRoute(size_t index, const AutoConfig &config) {
  if (!mappingValid())
    return nullptr;
  for (const PartRoute &route : partRoutes)
    if (destinationForPart(config, route.partId)) {
      if (!index)
        return &route;
      --index;
    }
  return nullptr;
}
inline void poseForBin(const RobotDHParams &params, size_t index, float pose[6]) {
  memcpy(pose, referencePose, sizeof(referencePose));
  if (index >= binCount) {
    pose[0] = pose[1] = pose[2] = NAN;
    return;
  }
  // The roll axis is the Z axis after DH joint 1, expressed in the base.
  // At shoulder pitch=0 it is vertical through (-76,0) in the current model.
  float homeRadians[5]{};
  BLA::Matrix<4, 4> frames[5], tcp;
  robot_fkine_all(homeRadians, params, frames, tcp);
  BLA::Matrix<4, 4> rollFrame = params.T_B0 * frames[0];
  float axis[3], displacement[3];
  float norm = hypotf(hypotf(rollFrame(0, 2), rollFrame(1, 2)), rollFrame(2, 2));
  if (!isfinite(norm) || norm < 1e-6f) {
    pose[0] = pose[1] = pose[2] = NAN;
    return;
  }
  for (int i = 0; i < 3; ++i) {
    axis[i] = rollFrame(i, 2) / norm;
    displacement[i] = referencePose[i] - rollFrame(i, 3);
  }
  float angle = bins[index].rollOffsetDegrees * PI / 180;
  float c = cosf(angle), s = sinf(angle);
  float dot = axis[0] * displacement[0] + axis[1] * displacement[1] +
              axis[2] * displacement[2];
  float cross[3] = {axis[1] * displacement[2] - axis[2] * displacement[1],
                    axis[2] * displacement[0] - axis[0] * displacement[2],
                    axis[0] * displacement[1] - axis[1] * displacement[0]};
  for (int i = 0; i < 3; ++i)
    pose[i] = rollFrame(i, 3) + c * displacement[i] + s * cross[i] +
              (1 - c) * dot * axis[i];
  // Rotate the reference orientation by the SAME base-frame axis rotation.
  // For the current vertical ring this preserves roll/pitch and adds the bin
  // offset to yaw; use matrices so a changed DH axis is handled consistently.
  const float roll=referencePose[3]*PI/180,pitch=referencePose[4]*PI/180,yaw=referencePose[5]*PI/180;
  const float cr=cosf(roll),sr=sinf(roll),cp=cosf(pitch),sp=sinf(pitch),cy=cosf(yaw),sy=sinf(yaw);
  const float reference[3][3]={{cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr},
    {sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr},{-sp,cp*sr,cp*cr}};
  float rotated[3][3];
  for(int col=0;col<3;++col) {
    float projection=0;for(int row=0;row<3;++row)projection+=axis[row]*reference[row][col];
    const float crossColumn[3]={axis[1]*reference[2][col]-axis[2]*reference[1][col],
      axis[2]*reference[0][col]-axis[0]*reference[2][col],
      axis[0]*reference[1][col]-axis[1]*reference[0][col]};
    for(int row=0;row<3;++row)
      rotated[row][col]=c*reference[row][col]+s*crossColumn[row]+(1-c)*projection*axis[row];
  }
  const float resultPitch=atan2f(-rotated[2][0],hypotf(rotated[0][0],rotated[1][0]));
  float resultRoll,resultYaw;
  if(fabsf(cosf(resultPitch))>1e-5f) {
    resultRoll=atan2f(rotated[2][1],rotated[2][2]);resultYaw=atan2f(rotated[1][0],rotated[0][0]);
  } else {
    resultYaw=0;resultRoll=rotated[2][0]<0 ? atan2f(rotated[0][1],rotated[0][2]):
                                                        atan2f(-rotated[0][1],-rotated[0][2]);
  }
  pose[3]=resultRoll*180/PI;pose[4]=resultPitch*180/PI;pose[5]=resultYaw*180/PI;
}
inline void configure(AutoConfig &config, const RobotDHParams &params) {
  for (size_t i = 0; i < binCount; ++i) {
    config.destinations[i].binId = bins[i].binId;
    config.destinations[i].enabled = bins[i].enabled;
    poseForBin(params, i, config.destinations[i].pose);
  }
  // ID-ignorant operation uses the original, farthest bin.
  memcpy(config.defaultDestination, config.destinations[2].pose, sizeof(config.defaultDestination));
  config.classificationPitchZero = true;
  config.classificationFixedShoulders = true;
  config.holdPickupQ5ForSorting = false; // q3/q4/q5 may move to enforce the bin orientation.
}
} // namespace Classification
