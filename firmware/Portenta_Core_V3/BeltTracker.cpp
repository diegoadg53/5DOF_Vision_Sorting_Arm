#include "BeltTracker.h"

void BeltTracker::clear() {
  for (Part &p : pool_)
    p = Part{};
}
bool BeltTracker::setVelocity(float v) {
  clear();
  calibrated_ = isfinite(v) && v < -.1f;
  velocity_ = calibrated_ ? v : 0;
  return calibrated_;
}
bool BeltTracker::setCameraXOffsetMm(float mm) {
  return setCameraXOffsetsMm(mm,mm);
}
bool BeltTracker::setCameraXOffsetsMm(float wide,float tall) {
  if(!isfinite(wide) || !isfinite(tall))return false;
  if(wide!=cameraXOffsetMm_[0] || tall!=cameraXOffsetMm_[1])clear();
  cameraXOffsetMm_[0]=wide;cameraXOffsetMm_[1]=tall;
  return true;
}
void BeltTracker::tick(uint32_t now) {
  if (!calibrated_)
    return;
  for (Part &p : pool_)
    if (p.inUse) {
      uint32_t dt = now - p.stateMs;
      p.y += velocity_ * (dt / 1000.0f);
      p.stateMs = now;
      if (!p.reserved && (p.y < -50 || now - p.observedMs > 15000))
        p = Part{};
    }
}
void BeltTracker::measure(Part &p, const objectVision &d, float x, float y, uint32_t now) {
  if (!p.hits) {
    p.cameraX = x;
    p.y = y;
    p.stateMs = now;
  } else {
    p.cameraX += .5f * (x - p.cameraX);
    p.y += .5f * (y - p.y);
  }
  updateClassification(p, d, now);
  p.boxWidth = d.width;
  p.boxHeight = d.height;
  p.x=p.cameraX+cameraXOffsetMm(d.height>d.width);
  if (p.hits < 65535)
    ++p.hits;
  p.observedMs = now;
  p.stateMs = now;
}
void BeltTracker::updateClassification(Part &p, const objectVision &d, uint32_t now) {
  const bool moved = d.xCenter != p.classPixelX || d.yCenter != p.classPixelY;
  const bool changed = !p.hits || d.ID != p.pendingClass;
  if (!p.hits) p.classId = d.ID;
  if (changed || (moved && uint32_t(now - p.classLastMs) >
                              CameraTracking::ClassificationConfidence::maximumGapMs)) {
    p.pendingClass = d.ID;
    p.classVotes = 1;
    p.classSinceMs = p.classLastMs = now;
  } else if (moved && uint32_t(now - p.classLastMs) >=
                         CameraTracking::ClassificationConfidence::sampleIntervalMs) {
    if (p.classVotes < 255) ++p.classVotes;
    p.classLastMs = now;
  }
  // Remember input centres even when throttled: held camera results cannot
  // become new evidence merely because a timer expired. Box-size jitter also
  // cannot extend the measured stability span. A changed ID resets immediately.
  p.classPixelX = d.xCenter;
  p.classPixelY = d.yCenter;
  if (!p.reserved &&
      p.classVotes >= CameraTracking::ClassificationConfidence::minimumSamples &&
      uint32_t(p.classLastMs - p.classSinceMs) >=
          CameraTracking::ClassificationConfidence::minimumStableMs)
    p.classId = p.pendingClass;
}
void BeltTracker::updateFrame(const objectVision *d, int count, uint32_t now, const calibH &c) {
  if (!calibrated_ || !c.valid || count < 0 || count > 10 || (count && !d))
    return;
  // Process the observation time before advancing to the current loop time.
  tick(now);
  float x[10]{}, y[10]{};
  float predictedX[MAX_PARTS]{}, predictedY[MAX_PARTS]{};
  for (int j=0; j<MAX_PARTS; ++j) {
    predictedX[j] = pool_[j].cameraX; predictedY[j] = pool_[j].y;
  }
  bool valid[10]{}, usedDetection[10]{}, usedTrack[MAX_PARTS]{};
  for (int i = 0; i < count; ++i) {
    if (!region_.containsBox(d[i].xCenter, d[i].yCenter, d[i].width, d[i].height))
      continue;
    pixelTransformation(d[i].xCenter, d[i].yCenter, c, x[i], y[i]);
    // Associate in unshifted coordinates: changing box ratio must not create
    // a second track merely because its empirical X correction changes.
    const float correctedX=x[i]+cameraXOffsetMm(d[i].height>d[i].width);
    valid[i] = isfinite(x[i]) && isfinite(y[i]) && d[i].width > 0 && d[i].height > 0 && x[i] > 0 &&
               x[i] < 800 && isfinite(correctedX) && correctedX>0 && correctedX<800 && y[i] > -50 && y[i] < 800;
  }
  for (int assignment = 0; assignment < count; ++assignment) {
    float best = 1e30f;
    int ti = -1, di = -1;
    for (int i = 0; i < count; ++i)
      if (valid[i] && !usedDetection[i])
        for (int j = 0; j < MAX_PARTS; ++j)
          if (pool_[j].inUse && !usedTrack[j]) {
            float dx = x[i] - predictedX[j], dy = y[i] - predictedY[j];
            if (fabsf(dx) > 10 || fabsf(dy) > 15)
              continue;
            float cost = dx * dx / 100 + dy * dy / 225;
            if (cost < best) {
              best = cost;
              ti = j;
              di = i;
            }
          }
    if (ti < 0)
      break;
    usedDetection[di] = true;
    usedTrack[ti] = true;
    measure(pool_[ti], d[di], x[di], y[di], now);
  }
  for (int i = 0; i < count; ++i)
    if (valid[i] && !usedDetection[i]) {
      int slot = -1;
      for (int j = 0; j < MAX_PARTS; ++j)
        if (!pool_[j].inUse) {
          slot = j;
          break;
        }
      if (slot < 0) {
        ++dropped_;
        continue;
      }
      Part &p = pool_[slot];
      p = Part{};
      p.inUse = true;
      p.trackId = nextId_++;
      if (!nextId_)
        ++nextId_;
      measure(p, d[i], x[i], y[i], now);
    }
}
bool BeltTracker::candidate(int slot, Part &out, uint32_t now) const {
  if (slot < 0 || slot >= MAX_PARTS)
    return false;
  const Part &p = pool_[slot];
  if (!calibrated_ || !p.inUse || p.reserved || p.consumed || p.hits < 3 ||
      now - p.observedMs > 10000 || p.y <= 0 || p.y >= 800)
    return false;
  out = p;
  return true;
}
bool BeltTracker::reserve(uint32_t id) {
  for (Part &p : pool_)
    if (p.inUse && p.trackId == id && !p.reserved && !p.consumed) {
      p.reserved = true;
      return true;
    }
  return false;
}
bool BeltTracker::observation(uint32_t id, Part &out) const {
  for (const Part &p : pool_)
    if (p.inUse && p.trackId == id) {
      out = p;
      return true;
    }
  return false;
}
void BeltTracker::retire(uint32_t id) {
  for (Part &p : pool_)
    if (p.inUse && p.trackId == id) {
      // Keep matching consumed tracks until they leave, avoiding duplicate picks.
      p.reserved = false;
      p.consumed = true;
      return;
    }
}
void BeltCalibrator::reset() {
  outsideRegion_ = ignoredIds_ = repeatedPositions_ = 0;
  inputPositionSeen_ = false;
  lastInputPixelX_ = lastInputPixelY_ = 0;
  tagLocked_ = false;
  tagId_ = 0;
  firstMs_ = lastMs_ = count_ = 0;
  st_ = sy_ = stt_ = sty_ = syy_ = 0;
  firstX_ = firstY_ = lastX_ = lastY_ = 0;
  pixelX_ = pixelY_ = 0;
  inputX_ = inputY_ = NAN;
  issue_ = BeltCalibrationIssue::None;
}
bool BeltCalibrator::addFrame(const objectVision *tags, int count, uint32_t now, const calibH &c) {
  if (failed()) return false;
  if (count < 0 || count > 10 || (count && !tags)) {
    issue_ = BeltCalibrationIssue::InvalidFrame;
    return false;
  }
  int selected = -1;
  for (int i = 0; i < count; ++i) {
    const objectVision &tag = tags[i];
    if (!velocityRegion_.containsBox(tag.xCenter, tag.yCenter, tag.width, tag.height)) {
      ++outsideRegion_;
      continue;
    }
    // Unsigned interval also rejects IDs below minimumTagId by underflow.
    if (unsigned(tag.ID) - unsigned(CameraTracking::VelocityRegion::minimumTagId) >
        unsigned(CameraTracking::VelocityRegion::maximumTagId) -
            unsigned(CameraTracking::VelocityRegion::minimumTagId)) {
      ++ignoredIds_;
      continue;
    }
    if (selected >= 0) {
      issue_ = BeltCalibrationIssue::MultipleTags;
      return false;
    }
    selected = i;
  }
  if (selected < 0) return true; // Dropouts/corner-only frames do not refresh sample age.
  if (tagLocked_ && tags[selected].ID != tagId_) {
    issue_ = BeltCalibrationIssue::TagChanged;
    return false;
  }
  const unsigned before = count_;
  const bool ok = add(tags[selected], now, c);
  if (count_ > before && !tagLocked_) {
    tagId_ = tags[selected].ID;
    tagLocked_ = true;
  }
  return ok;
}
bool BeltCalibrator::add(const objectVision &d, uint32_t now, const calibH &c) {
  if (failed())
    return false;
  if (!velocityRegion_.containsBox(d.xCenter, d.yCenter, d.width, d.height)) {
    ++outsideRegion_;
    return true; // Ignore, without refreshing accepted-sample timing or sums.
  }
  // GET_RESULT can return the same held camera result on successive polls.
  // Another packet is not another independent position measurement. Compare
  // centres only: box-size jitter must not make a held centre look fresh.
  if (inputPositionSeen_ && d.xCenter == lastInputPixelX_ && d.yCenter == lastInputPixelY_) {
    ++repeatedPositions_;
    return true;
  }
  inputPositionSeen_ = true;
  lastInputPixelX_ = d.xCenter;
  lastInputPixelY_ = d.yCenter;
  // Mark new coordinates before throttling. If their FIRST arrival is too
  // soon, discard that position rather than accepting a repeat later with a
  // shifted timestamp. A later changed centre can still provide a sample.
  if (count_ && now - lastMs_ < BeltCalibrationLimits::sampleIntervalMs)
    return true;
  float x, y;
  pixelTransformation(d.xCenter, d.yCenter, c, x, y);
  x += cameraXOffsetMm_;
  pixelX_ = d.xCenter;
  pixelY_ = d.yCenter;
  inputX_ = x;
  inputY_ = y;
  if (!isfinite(x) || !isfinite(y)) {
    issue_ = BeltCalibrationIssue::NonfiniteProjection;
    return false;
  }
  if (count_) {
    if (fabsf(x - lastX_) > BeltCalibrationLimits::maximumLateralStepMm)
      issue_ = BeltCalibrationIssue::LateralJump;
    else if (y > lastY_ + BeltCalibrationLimits::maximumReverseStepMm)
      issue_ = BeltCalibrationIssue::ReverseStep;
    else if (now - lastMs_ > BeltCalibrationLimits::maximumGapMs)
      issue_ = BeltCalibrationIssue::SampleGap;
    if (failed())
      return false;
  }
  if (!count_) {
    firstMs_ = now;
    firstX_ = x;
    firstY_ = y;
  }
  double t = uint32_t(now - firstMs_) / 1000.0;
  ++count_;
  st_ += t;
  sy_ += y;
  stt_ += t * t;
  sty_ += t * y;
  syy_ += double(y) * y;
  lastMs_ = now;
  lastX_ = x;
  lastY_ = y;
  return true;
}
const char *belt_calibration_issue_name(BeltCalibrationIssue issue) {
  switch (issue) {
  case BeltCalibrationIssue::None: return "OK";
  case BeltCalibrationIssue::NonfiniteProjection: return "INVALID_HOMOGRAPHY_PROJECTION";
  case BeltCalibrationIssue::LateralJump: return "WORLD_X_JUMP";
  case BeltCalibrationIssue::ReverseStep: return "WORLD_Y_REVERSED";
  case BeltCalibrationIssue::SampleGap: return "SAMPLE_GAP";
  case BeltCalibrationIssue::TooFewSamples: return "TOO_FEW_SAMPLES";
  case BeltCalibrationIssue::ShortSpan: return "OBSERVATION_TOO_SHORT";
  case BeltCalibrationIssue::ShortTravel: return "TOO_LITTLE_NEGATIVE_Y_TRAVEL";
  case BeltCalibrationIssue::DegenerateTimes: return "DEGENERATE_SAMPLE_TIMES";
  case BeltCalibrationIssue::WrongDirection: return "VELOCITY_NOT_NEGATIVE_Y";
  case BeltCalibrationIssue::NoisyFit: return "REGRESSION_ERROR_TOO_HIGH";
  case BeltCalibrationIssue::MultipleTags: return "MULTIPLE_TAGS_INSIDE_ROI";
  case BeltCalibrationIssue::TagChanged: return "MOVING_TAG_ID_CHANGED";
  case BeltCalibrationIssue::InvalidFrame: return "INVALID_TAG_FRAME";
  }
  return "UNKNOWN";
}
BeltCalibrationReport BeltCalibrator::report() const {
  BeltCalibrationReport r;
  r.samples = count_;
  r.outsideRegion = outsideRegion_;
  r.ignoredIds = ignoredIds_;
  r.repeatedPositions = repeatedPositions_;
  r.tagLocked = tagLocked_;
  r.tagId = tagId_;
  r.spanMs = count_ ? uint32_t(lastMs_ - firstMs_) : 0;
  r.deltaXmm = count_ ? lastX_ - firstX_ : 0;
  r.travelMm = count_ ? firstY_ - lastY_ : 0; // Positive when travelling along -Y.
  r.pixelX = pixelX_;
  r.pixelY = pixelY_;
  r.worldX = inputX_;
  r.worldY = inputY_;
  double sxx = count_ ? stt_ - st_ * st_ / count_ : 0;
  double slope = NAN, rms = NAN;
  if (sxx > 1e-9) {
    double sxy = sty_ - st_ * sy_ / count_;
    slope = sxy / sxx;
    double sse = fmax(0.0, syy_ - sy_ * sy_ / count_ - slope * sxy);
    rms = sqrt(sse / count_);
    r.velocityMmS = float(slope);
    r.rmsMm = float(rms);
  }
  r.issue = issue_;
  if (r.issue == BeltCalibrationIssue::None) {
    if (count_ < BeltCalibrationLimits::minimumSamples)
      r.issue = BeltCalibrationIssue::TooFewSamples;
    else if (r.spanMs < BeltCalibrationLimits::minimumSpanMs)
      r.issue = BeltCalibrationIssue::ShortSpan;
    else if (r.travelMm < BeltCalibrationLimits::minimumTravelMm)
      r.issue = BeltCalibrationIssue::ShortTravel;
    else if (sxx <= 1e-9)
      r.issue = BeltCalibrationIssue::DegenerateTimes;
    else if (!isfinite(slope) || slope >= -.1)
      r.issue = BeltCalibrationIssue::WrongDirection;
    else if (rms > BeltCalibrationLimits::maximumRmsMm)
      r.issue = BeltCalibrationIssue::NoisyFit;
  }
  return r;
}
bool BeltCalibrator::finish(float &v) const {
  BeltCalibrationReport r = report();
  if (r.issue != BeltCalibrationIssue::None)
    return false;
  v = r.velocityMmS;
  return true;
}
