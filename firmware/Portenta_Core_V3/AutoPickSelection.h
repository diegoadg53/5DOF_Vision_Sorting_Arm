#pragma once
#include "PickMotionConfig.h"
#include "ClassificationConfig.h"
#include "RuntimeTuning.h"

namespace AutoPick {
enum class Admission : uint8_t { Eligible, AwaitingClassification, IgnoredId, OutsidePickupRange, TooLate, Invalid };
struct Request {
  Part part;
  AutoConfig config;
  float destination[6]{};
  uint8_t binId = 0;
  double closingDelayMs = 0;
  double arrivalDelayMs = 0;
  float idExtraXmm = 0;
  PickupProfilePlan profiles;
};

inline Admission evaluate(const Part &part, float velocity, const AutoConfig &base,
                          bool respectId, uint32_t now, Request &out,const RuntimeTuning *tuning=nullptr) {
  if (!isfinite(velocity) || velocity >= -.1f || !isfinite(part.x) || !isfinite(part.y))
    return Admission::Invalid;
  const bool stable = partClassificationStable(part);
  const Destination *mapped = respectId && stable ? Classification::destinationForPart(base, part.classId)
                                        : nullptr;
  const bool tall=part.boxHeight>part.boxWidth;
  const float extra=respectId && stable && tuning ? tuning->extraXmm(part.classId,tall):0;
  const float pickupX=part.x+extra;
  // An unknown TALL label may later become ID 1 and bring an edge observation
  // into range. Wait for classification only if either possible X is valid.
  const bool pendingMayEnterRange=respectId && !stable && tuning && tall &&
      PickMotion::supportsX(part.x+tuning->extraXmm(1,true));
  if (!PickMotion::supportsX(pickupX) && !pendingMayEnterRange)
    return Admission::OutsidePickupRange;
  // Do not retire an early unknown/unmapped label before it can be corrected.
  if (respectId && stable && !mapped)
    return Admission::IgnoredId;
  // Track snapshots normally have stateMs=now. Account for age as well so this
  // helper never grants extra preparation time to an older observation.
  double delay = -double(part.y) / double(velocity) * 1000.0 -
                 double(uint32_t(now - part.stateMs));
  AutoConfig selected=base;
  if(tuning)tuning->apply(selected,part.boxHeight>part.boxWidth ? 1:0);
  if (delay < selected.pickUpBudgetMs())
    return Admission::TooLate;
  if (respectId && !stable)
    return Admission::AwaitingClassification;
  out.part = part;
  out.part.x = pickupX;
  out.idExtraXmm = extra;
  out.config = selected; // Preserve enabled bins and operator timing settings.
  PickMotion::apply(out.config, pickupX);
  out.binId = mapped ? mapped->binId : Classification::bins[Classification::binCount - 1].binId;
  memcpy(out.destination, mapped ? mapped->pose : base.defaultDestination, sizeof(out.destination));
  out.arrivalDelayMs = delay;
  out.closingDelayMs = delay - selected.closingLeadMs;
  if(tuning)out.profiles=tuning->plans(base,part.cameraX,true,respectId && stable ? part.classId:0);
  return Admission::Eligible;
}
} // namespace AutoPick
