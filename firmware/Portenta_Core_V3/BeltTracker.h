#pragma once
#include <Arduino.h>
#include <math.h>
#include "VisionPacket.h"
#include "DOF5_wServo_Kinematics.h"
#include "CameraTrackingConfig.h"

constexpr int MAX_PARTS = 50;
struct Part {
  uint32_t trackId = 0, stateMs = 0, observedMs = 0;
  float x = 0, y = 0, cameraX = 0;
  uint16_t hits = 0;
  uint8_t classId = 0, pendingClass = 0, classVotes = 0;
  uint32_t classSinceMs = 0, classLastMs = 0;
  uint16_t classPixelX = 0, classPixelY = 0;
  uint16_t boxWidth = 0, boxHeight = 0;
  bool inUse = false, reserved = false, consumed = false;
};

inline bool partClassificationStable(const Part &p) {
  return p.classId == p.pendingClass &&
         p.classVotes >= CameraTracking::ClassificationConfidence::minimumSamples &&
         uint32_t(p.classLastMs - p.classSinceMs) >=
             CameraTracking::ClassificationConfidence::minimumStableMs;
}

class BeltTracker {
public:
  explicit BeltTracker(float cameraXOffsetMm = CameraTracking::partXOffsetMm,
                       CameraTracking::PixelRectangle region = CameraTracking::PartRegion::rectangle)
      : cameraXOffsetMm_{cameraXOffsetMm,cameraXOffsetMm}, region_(region) {}
  void clear();
  bool setVelocity(float mm_s);
  // Change only at rest; discard tracks expressed using the previous offset.
  bool setCameraXOffsetMm(float mm);
  bool setCameraXOffsetsMm(float wide,float tall);
  float cameraXOffsetMm(bool tall=false) const { return cameraXOffsetMm_[tall ? 1:0]; }
  float velocity() const {
    return velocity_;
  }
  bool calibrated() const {
    return calibrated_;
  }
  void tick(uint32_t now);
  void updateFrame(const objectVision *detections, int count, uint32_t timestamp,
                   const calibH &calibration);
  bool candidate(int slot, Part &out, uint32_t now) const;
  bool reserve(uint32_t id);
  bool observation(uint32_t id, Part &out) const;
  void retire(uint32_t id);
  uint32_t dropped() const {
    return dropped_;
  }

private:
  // clear()/setVelocity() retain the correction; its setter clears old tracks.
  float cameraXOffsetMm_[2];
  CameraTracking::PixelRectangle region_;
  Part pool_[MAX_PARTS]{};
  float velocity_ = 0;
  uint32_t nextId_ = 1, dropped_ = 0;
  bool calibrated_ = false;
  void measure(Part &p, const objectVision &d, float x, float y, uint32_t now);
  void updateClassification(Part &p, const objectVision &d, uint32_t now);
};

namespace BeltCalibrationLimits {
// Sparse small-tag detections: fewer samples, but retain span/travel/fit gates.
constexpr unsigned minimumSamples = 4;
constexpr uint32_t sampleIntervalMs = 80, minimumSpanMs = 800, maximumGapMs = 1500;
constexpr float minimumTravelMm = 20, maximumLateralStepMm = 20, maximumReverseStepMm = 10;
// Operator-approved repeatability trial: previous 5 mm limit rejected a
// 28-sample/2698 ms pass with 7.13 mm RMS. This is a fit-quality allowance.
constexpr float maximumRmsMm = 8;
}
enum class BeltCalibrationIssue : uint8_t {
  None, NonfiniteProjection, LateralJump, ReverseStep, SampleGap,
  TooFewSamples, ShortSpan, ShortTravel, DegenerateTimes, WrongDirection, NoisyFit,
  MultipleTags, TagChanged, InvalidFrame
};
const char *belt_calibration_issue_name(BeltCalibrationIssue issue);
struct BeltCalibrationReport {
  BeltCalibrationIssue issue = BeltCalibrationIssue::TooFewSamples;
  unsigned samples = 0;
  uint32_t outsideRegion = 0;
  uint32_t ignoredIds = 0;
  uint32_t repeatedPositions = 0;
  bool tagLocked = false;
  uint8_t tagId = 0;
  uint32_t spanMs = 0;
  float deltaXmm = 0, travelMm = 0, velocityMmS = NAN, rmsMm = NAN;
  // Last evaluated detection, including one rejected by a continuity check.
  uint16_t pixelX = 0, pixelY = 0;
  float worldX = NAN, worldY = NAN;
};
class BeltCalibrator {
public:
  explicit BeltCalibrator(float cameraXOffsetMm = 0,
                          CameraTracking::PixelRectangle velocityRegion = CameraTracking::VelocityRegion::rectangle)
      : cameraXOffsetMm_(cameraXOffsetMm), velocityRegion_(velocityRegion) {}
  void reset();
  bool add(const objectVision &d, uint32_t now, const calibH &calibration);
  // Select by pixel geometry BEFORE ID: fixed/moving tags may share IDs 0..3.
  bool addFrame(const objectVision *tags, int count, uint32_t now, const calibH &calibration);
  bool finish(float &velocity) const;
  BeltCalibrationReport report() const;
  unsigned samples() const {
    return count_;
  }
  bool seen() const {
    return count_ > 0;
  }
  uint32_t lastSampleMs() const {
    return lastMs_;
  }
  bool failed() const {
    return issue_ != BeltCalibrationIssue::None;
  }

private:
  float cameraXOffsetMm_;
  CameraTracking::PixelRectangle velocityRegion_;
  uint32_t outsideRegion_ = 0;
  uint32_t ignoredIds_ = 0;
  uint32_t repeatedPositions_ = 0;
  uint16_t lastInputPixelX_ = 0, lastInputPixelY_ = 0;
  bool inputPositionSeen_ = false;
  bool tagLocked_ = false;
  uint8_t tagId_ = 0;
  uint32_t firstMs_ = 0, lastMs_ = 0;
  unsigned count_ = 0;
  double st_ = 0, sy_ = 0, stt_ = 0, sty_ = 0, syy_ = 0;
  float firstX_ = 0, firstY_ = 0, lastX_ = 0, lastY_ = 0;
  uint16_t pixelX_ = 0, pixelY_ = 0;
  float inputX_ = NAN, inputY_ = NAN;
  BeltCalibrationIssue issue_ = BeltCalibrationIssue::None;
};
