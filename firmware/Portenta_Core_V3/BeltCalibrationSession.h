#pragma once
#include "BeltTracker.h"

namespace BeltCalibrationSessionConfig {
// EDIT HERE: independent accepted passes, not detections within one pass.
constexpr unsigned requiredRuns = 10;
static_assert(requiredRuns >= 10, "Belt calibration requires at least ten valid runs");
constexpr uint32_t laneClearMs = 1500, maximumClearFrameGapMs = 500;
constexpr uint32_t passTimeoutMs = 20000;
}

// RAM-only session. No partially completed session becomes AUTO's belt speed.
struct BeltCalibrationSession {
  enum class Phase : uint8_t { Clearing, Ready, Recording };
  Phase phase = Phase::Clearing;
  unsigned accepted = 0, rejected = 0;
  double mean = 0, m2 = 0;
  float minimum = NAN, maximum = NAN;
  uint32_t clearSinceMs = 0, clearFrameMs = 0;
  bool clearing = false;
  bool cancelRequested = false;

  void waitForClear() { phase = Phase::Clearing; clearing = false; }
  bool observeClear(bool occupied, uint32_t now) {
    if (occupied) { clearing = false; return false; }
    if (!clearing || uint32_t(now - clearFrameMs) >
                         BeltCalibrationSessionConfig::maximumClearFrameGapMs) {
      clearing = true;
      clearSinceMs = now;
    }
    clearFrameMs = now;
    if (uint32_t(now - clearSinceMs) < BeltCalibrationSessionConfig::laneClearMs)
      return false;
    phase = Phase::Ready;
    return true;
  }
  bool record(float velocity) {
    if (!isfinite(velocity) || velocity >= -.1f ||
        accepted >= BeltCalibrationSessionConfig::requiredRuns) return false;
    if (!accepted) minimum = maximum = velocity;
    if (velocity < minimum) minimum = velocity;
    if (velocity > maximum) maximum = velocity;
    ++accepted;
    const double delta = velocity - mean;
    mean += delta / accepted;
    m2 += delta * (velocity - mean);
    return true;
  }
  float standardDeviation() const {
    return accepted > 1 ? float(sqrt(fmax(0.0, m2) / (accepted - 1))) : 0;
  }
  bool complete() const { return accepted == BeltCalibrationSessionConfig::requiredRuns; }
};
