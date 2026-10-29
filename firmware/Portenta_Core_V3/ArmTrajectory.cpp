#include "ArmTrajectory.h"
#include <string.h>

namespace {
// At shoulder pitch=0, select the ring's radial shoulder-roll angle. Derive
// its zero direction and axis from FK rather than embedding the base offset.
bool sortingShoulderRoll(const RobotDHParams &p,const float pose[6],float &degrees) {
  float zero[5]{};BLA::Matrix<4,4> frames[5],tcp;
  robot_fkine_all(zero,p,frames,tcp);
  const BLA::Matrix<4,4> frame=p.T_B0*frames[0];
  float axis[3],home[3],desired[3];
  const float norm=hypotf(hypotf(frame(0,2),frame(1,2)),frame(2,2));
  if(!isfinite(norm) || norm<1e-6f)return false;
  float homeDot=0,desiredDot=0;
  for(int i=0;i<3;++i) {
    axis[i]=frame(i,2)/norm;home[i]=tcp(i,3)-frame(i,3);desired[i]=pose[i]-frame(i,3);
    homeDot+=axis[i]*home[i];desiredDot+=axis[i]*desired[i];
  }
  float homeNorm=0,desiredNorm=0,dot=0;
  for(int i=0;i<3;++i) {
    home[i]-=homeDot*axis[i];desired[i]-=desiredDot*axis[i];
    homeNorm+=home[i]*home[i];desiredNorm+=desired[i]*desired[i];dot+=home[i]*desired[i];
  }
  if(!isfinite(homeNorm) || !isfinite(desiredNorm) || homeNorm<1e-6f || desiredNorm<1e-6f)return false;
  const float cross[3]={home[1]*desired[2]-home[2]*desired[1],home[2]*desired[0]-home[0]*desired[2],
                        home[0]*desired[1]-home[1]*desired[0]};
  const float angle=atan2f(axis[0]*cross[0]+axis[1]*cross[1]+axis[2]*cross[2],dot)*180/PI;
  for(int turn=0;turn<3;++turn) {
    const float candidate=angle+(turn==1 ? 360:turn==2 ? -360:0),rad=candidate*PI/180;
    if(isfinite(rad) && rad>=p.q_min[1] && rad<=p.q_max[1]) {degrees=candidate;return true;}
  }
  return false;
}
}

bool validPickupTiming(const AutoConfig &c) {
  // Bound arithmetic and wrap-safe elapsed comparisons to a practical range.
  const uint32_t limit = 60000;
  return c.descentLeadMs && c.descentLeadMs < limit &&
         c.closingLeadMs <= c.descentLeadMs &&
         c.closingDurationMs >= c.closingLeadMs && c.closingDurationMs < limit &&
         c.pickupPostCloseMs < limit &&
         c.pickUpMarginMs && c.pickUpMarginMs < limit &&
         c.scheduleToleranceMs < c.descentLeadMs && c.motionTimeoutMs &&
         c.openingDurationMs && c.settleMs && c.feedbackTimeoutMs &&
         isfinite(c.maximumCaptureErrorYmm) && c.maximumCaptureErrorYmm > 0;
}

bool validArrivalConfiguration(const AutoConfig &c) {
  if (!isfinite(c.jointToleranceDeg) || c.jointToleranceDeg <= 0 ||
      !isfinite(c.positionToleranceMm) || c.positionToleranceMm <= 0)
    return false;
  for (float tolerance : c.jointToleranceOverrideDeg)
    if (!isfinite(tolerance) || tolerance < 0)
      return false;
  for (int i=0; i<5; ++i) {
    const float extra=c.jointHoldHysteresisDeg[i];
    const float entry=c.useHomeWaypointArrival ? c.homeWaypointArrival.jointToleranceDeg[i] :
        (c.jointToleranceOverrideDeg[i]>0 ? c.jointToleranceOverrideDeg[i] : c.jointToleranceDeg);
    if (!isfinite(extra) || extra<0 || !isfinite(entry+extra)) return false;
    const float normal=c.jointToleranceOverrideDeg[i]>0 ? c.jointToleranceOverrideDeg[i] : c.jointToleranceDeg;
    if (!isfinite(normal+extra)) return false;
  }
  if (!c.useHomeWaypointArrival)
    return true;
  const ArrivalTolerance &a = c.homeWaypointArrival;
  for (float tolerance : a.jointToleranceDeg)
    if (!isfinite(tolerance) || tolerance <= 0)
      return false;
  return isfinite(a.positionToleranceMm) && a.positionToleranceMm > 0 &&
         isfinite(a.positionHysteresisMm) && a.positionHysteresisMm >= 0 &&
         isfinite(a.positionToleranceMm + a.positionHysteresisMm);
}

void ArmTrajectory::resetDiagnostics(const ArmFeedback &f, uint32_t now) {
  diagnosticRead_ = diagnosticCount_ = 0;
  diagnosticTickMs_ = now;
  diagnosticFeedbackMs_ = f.timestampMs;
  diagnosticSequence_ = f.sequence;
  tickGapMs_ = maximumTickGapMs_ = feedbackIntervalMs_ = 0;
}

bool ArmTrajectory::takeDiagnostic(ArmDiagnosticReport &report) {
  if (!diagnosticCount_) return false;
  report = diagnostics_[diagnosticRead_];
  diagnosticRead_ = (diagnosticRead_ + 1) % 8;
  --diagnosticCount_;
  return true;
}

void ArmTrajectory::captureDiagnostic(ArmDiagnosticEvent event, const ArmFeedback &f,
                                      uint32_t now, const char *reason) {
  ArmDiagnosticReport d;
  d.event = event;
  d.reason = reason;
  d.stage = phaseName();
  d.targetName = phase_ == Phase::Above || phase_ == Phase::SafetyWait ? "SAFETY_HEIGHT" :
                 phase_ == Phase::Down || phase_ == Phase::Wait || phase_ == Phase::Close ?
                     "GRIPPING_HEIGHT" : phaseName();
  d.trackId = id_;
  d.progress = progress(f, now);
  d.remainingArrivalMs = arrivalDelayMs(now);
  d.settleMs = config_.settleMs;
  d.minimumMoveMs = config_.minimumMoveMs;
  d.marginMs = config_.pickUpMarginMs;
  d.descentLeadMs = config_.descentLeadMs;
  d.closingLeadMs = config_.closingLeadMs;
  d.closingDurationMs = config_.closingDurationMs;
  d.budgetMs = config_.pickUpBudgetMs();
  d.closingStarted = closeIssued_;
  d.closeElapsedMs = closeIssued_ ? uint32_t(now - closeMs_) : 0;
  d.timedPickup = usesTimedPickup();
  d.tallPickup = selectedVariant_ != 0;
  d.postCloseMs = d.timedPickup ? config_.pickupPostCloseMs : 0;
  if (event == ArmDiagnosticEvent::SafetyConfirmed || !strcmp(reason, "SAFETY_HEIGHT_LATE") ||
      !strcmp(reason,"BOX_PROFILE_CHANGED_LATE")) {
    d.requirement = "SAFETY_CONFIRMED_BEFORE_DESCENT";
    d.thresholdMs = config_.descentLeadMs;
    d.hasDeadline = true;
  } else if (event == ArmDiagnosticEvent::GrippingConfirmed || !strcmp(reason, "GRIPPING_HEIGHT_LATE")) {
    d.requirement = "GRIPPING_CONFIRMED_BY_Y0";
    d.hasDeadline = true; // Threshold is zero; no descent lead required again.
  } else if (event == ArmDiagnosticEvent::DescentCommand || !strcmp(reason, "DESCENT_COMMAND_LATE")) {
    d.requirement = "DESCENT_START";
    d.thresholdMs = config_.descentLeadMs;
    d.allowedLateMs = config_.scheduleToleranceMs;
    d.hasDeadline = true;
  } else if (event == ArmDiagnosticEvent::CloseCommand || !strcmp(reason, "CLOSE_COMMAND_LATE")) {
    d.requirement = "CLOSE_START";
    d.thresholdMs = config_.closingLeadMs;
    d.allowedLateMs = allowedLateness();
    d.hasDeadline = true;
  } else if (!strcmp(reason, "PLAN_LATE") || !strcmp(reason, "APPROACH_COMMAND_LATE")) {
    d.requirement = "APPROACH_START";
    d.thresholdMs = config_.pickUpBudgetMs();
    d.allowedLateMs = config_.scheduleToleranceMs;
    d.hasDeadline = true;
  }
  if (diagnosticCount_ == 8) {
    // Preserve recent/failure evidence if a caller does not drain diagnostics.
    // The motion state machine never waits for console output.
    diagnosticRead_ = (diagnosticRead_ + 1) % 8;
    --diagnosticCount_;
  }
  diagnostics_[(diagnosticRead_ + diagnosticCount_) % 8] = d;
  ++diagnosticCount_;
}

void ArmTrajectory::requestPause(uint32_t now) {
  if (!busy() || paused_ || pauseRequested_)
    return;
  pauseRequested_ = true;
  pauseRequestedMs_ = now;
}

bool ArmTrajectory::pauseAtBoundary(uint32_t now) {
  if (!pauseRequested_)
    return false;
  paused_ = true;
  pauseRequested_ = false;
  pauseBoundaryMs_ = now;
  return true;
}

bool ArmTrajectory::resume(uint32_t now) {
  if (!paused_)
    return false;
  // Shift only virtual part/stage clocks. Telemetry timestamps stay real.
  part_.stateMs += uint32_t(now - pauseRequestedMs_);
  stageMs_ += uint32_t(now - pauseBoundaryMs_);
  paused_ = pauseRequested_ = false;
  return true;
}

const char *ArmTrajectory::phaseName() const {
  switch (phase_) {
  case Phase::Idle:
    return "IDLE";
  case Phase::Plan:
    return "PLAN";
  case Phase::Schedule:
    return "WAIT FOR APPROACH";
  case Phase::Home:
    return "HOME";
  case Phase::Above:
    return "APPROACH";
  case Phase::SafetyWait:
    return "SAFETY HEIGHT WAIT";
  case Phase::Down:
    return "DESCEND";
  case Phase::Wait:
    return "WAIT FOR PART";
  case Phase::Close:
    return "CLOSE";
  case Phase::Mid:
    return "CLASSIFICATION WAYPOINT";
  case Phase::Drop:
    return "CLASSIFICATION POSE";
  case Phase::Open:
    return "OPEN";
  case Phase::Recover:
    return "MISSED-PICK RECOVERY";
  case Phase::RecoverWait:
    return "RECOVERY CLOSE WAIT";
  case Phase::RecoverHome:
    return "RECOVERY HOME";
  }
  return "UNKNOWN";
}

bool ArmTrajectory::healthy(const ArmFeedback &f, uint32_t now) const {
  if (!params_ || !io_.joints || !io_.gripper || !f.valid ||
      now - f.timestampMs > config_.feedbackTimeoutMs)
    return false;
  for (float v : f.q)
    if (!isfinite(v))
      return false;
  return true;
}
float ArmTrajectory::jointTolerance(int joint, bool holding) const {
  float perJoint = config_.jointToleranceOverrideDeg[joint];
  const float entry = homeWaypointArrivalActive_ ? config_.homeWaypointArrival.jointToleranceDeg[joint] :
                      perJoint > 0 ? perJoint : config_.jointToleranceDeg;
  return entry + (holding ? config_.jointHoldHysteresisDeg[joint] : 0);
}
float ArmTrajectory::positionTolerance(bool holding) const {
  if (!homeWaypointArrivalActive_)
    return config_.positionToleranceMm;
  const ArrivalTolerance &a = config_.homeWaypointArrival;
  return a.positionToleranceMm + (holding ? a.positionHysteresisMm : 0);
}
bool ArmTrajectory::atTarget(const ArmFeedback &f, bool holding) const {
  for (int i = 0; i < 5; ++i)
    if (fabsf(f.q[i] - target_[i]) > jointTolerance(i, holding && targetConfirmed_))
      return false;
  float a[6], b[6];
  robot_fkine(f.q, *params_, a);
  robot_fkine(target_, *params_, b);
  return hypotf(hypotf(a[0] - b[0], a[1] - b[1]), a[2] - b[2]) <= positionTolerance(holding);
}
ArmProgress ArmTrajectory::progress(const ArmFeedback &f, uint32_t now) const {
  ArmProgress p;
  p.elapsedMs = uint32_t((paused_ ? pauseBoundaryMs_ : now) - stageMs_);
  p.feedbackAgeMs = uint32_t(now - f.timestampMs);
  p.tickGapMs = tickGapMs_;
  p.maximumTickGapMs = maximumTickGapMs_;
  p.feedbackIntervalMs = feedbackIntervalMs_;
  p.feedbackValid = healthy(f, now);
  p.settlingStarted = settling_;
  p.jointHoldActive = targetConfirmed_;
  memcpy(p.target, target_, sizeof(p.target));
  p.positionEntryToleranceMm = positionTolerance(false);
  p.positionHoldToleranceMm = positionTolerance(true);
  for (int i = 0; i < 5; ++i) {
    p.error[i] = f.q[i] - target_[i];
    p.jointEntryToleranceDeg[i] = jointTolerance(i);
    p.jointHoldToleranceDeg[i] = jointTolerance(i, true);
    p.jointToleranceDeg[i] = jointTolerance(i, targetConfirmed_);
  }
  if (p.feedbackValid) {
    float actual[6], expected[6];
    robot_fkine(f.q, *params_, actual);
    robot_fkine(target_, *params_, expected);
    p.positionErrorMm =
        hypotf(hypotf(actual[0] - expected[0], actual[1] - expected[1]), actual[2] - expected[2]);
    p.withinEntry = atTarget(f);
    p.withinTarget = atTarget(f, settling_);
    if (settling_ && p.withinTarget)
      p.stableMs = uint32_t(f.timestampMs - settledSince_);
  }
  return p;
}
bool ArmTrajectory::arrived(const ArmFeedback &f, uint32_t now) {
  if (f.sequence != lastSequence_) {
    lastSequence_ = f.sequence;
    if (atTarget(f, settling_)) {
      if (!settling_) {
        settling_ = true;
        settledSince_ = f.timestampMs;
      }
    } else
      settling_ = targetConfirmed_ = false;
  }
  const bool ready = settling_ && f.timestampMs - settledSince_ >= config_.settleMs &&
                     now - stageMs_ >= config_.minimumMoveMs;
  if (ready) targetConfirmed_ = true;
  return ready;
}
bool ArmTrajectory::move(const float q[5], Phase phase, uint32_t now) {
  for (int i = 0; i < 5; ++i) {
    // Match the float-radian representation used by IK and stored limits.
    const float radians = q[i] * PI / 180;
    if (!isfinite(q[i]) || radians < params_->q_min[i] || radians > params_->q_max[i]) {
      fail("Joint target outside limits");
      return false;
    }
  }
  if (!io_.joints(q)) {
    fail("Position command failed");
    return false;
  }
  memcpy(target_, q, sizeof(target_));
  homeWaypointArrivalActive_ = config_.useHomeWaypointArrival &&
                              (phase == Phase::Home || phase == Phase::Mid || phase == Phase::RecoverHome);
  phase_ = phase;
  stageMs_ = now;
  settling_ = false;
  targetConfirmed_ = false;
  return true;
}
bool ArmTrajectory::prepare(const AutoConfig &c, const ArmFeedback &f, uint32_t now, bool keepClosed) {
  if (busy() || outcome_ != PickOutcome::None)
    return false;
  config_ = c;
  classificationReportPending_=false;
  profilePlanActive_=profileReportPending_=false;
  targetConfirmed_ = false;
  homeWaypointArrivalActive_ = c.useHomeWaypointArrival;
  id_ = 0;
  fault_ = "";
  faultStage_ = "";
  rejectionReason_ = "";
  paused_ = pauseRequested_ = false;
  closeIssued_ = homeKeepClosed_ = keepClosed;
  missReason_ = "";
  resetDiagnostics(f, now);
  orientationReportPending_ = false;
  if (!validArrivalConfiguration(c)) {
    fail("Invalid arrival tolerances");
    return false;
  }
  if (!healthy(f, now)) {
    fail("No fresh Nucleo feedback");
    return false;
  }
  lastSequence_ = f.sequence;
  const float home[5] = {0, 0, 0, 0, 0};
  if (io_.home) {
    memcpy(target_, home, sizeof(target_));
    phase_ = Phase::Home;
    stageMs_ = now;
    settling_ = false;
    for (int i = 0; i < 5; ++i)
      if (params_->q_min[i] > 0 || params_->q_max[i] < 0) {
        fail("Joint target outside limits");
        return false;
      }
    // The adapter validates targets and writes the ordered newline-delimited
    // batch once, compatible with both current and older Nucleo receivers.
    if (!io_.home(home, keepClosed ? config_.closeAngle : config_.openAngle)) {
      fail("HOME command failed");
      return false;
    }
    return true;
  }
  if (!move(home, Phase::Home, now))
    return false;
  if (!io_.gripper(keepClosed ? config_.closeAngle : config_.openAngle)) {
    fail("Open command failed");
    return false;
  }
  return true;
}
double ArmTrajectory::arrivalDelayMs(uint32_t now) const {
  // Use the assigned snapshot's time and signed velocity. Negative delay
  // means the close-command deadline has passed; never cast it to unsigned.
  uint32_t partTime = (paused_ || pauseRequested_) ? pauseRequestedMs_ : now;
  return -double(part_.y) / double(velocity_) * 1000.0 -
         double(uint32_t(partTime - part_.stateMs));
}
double ArmTrajectory::closingDelay(uint32_t now) const {
  return arrivalDelayMs(now) - config_.closingLeadMs;
}

void ArmTrajectory::updateObservation(const Part &p) {
  if (!busy() || orientationLocked_ || p.trackId != id_ || !p.boxWidth || !p.boxHeight)
    return;
  // Only the box/profile remains live. Never replace reserved camera X/time/ID/bin.
  part_.boxWidth = p.boxWidth;
  part_.boxHeight = p.boxHeight;
}
bool ArmTrajectory::takeClassificationPlan(ClassificationPlanReport &report) {
  if(!classificationReportPending_)return false;
  classificationReportPending_=false;
  report.trackId=id_;report.fixedShoulders=config_.classificationFixedShoulders;
  report.heldPickupQ5=config_.holdPickupQ5ForSorting;
  memcpy(report.requested,destination_,sizeof(report.requested));
  memcpy(report.joints,drop_,sizeof(report.joints));robot_fkine(drop_,*params_,report.planned);
  report.ik=classificationIk_[selectedVariant_];return true;
}

void ArmTrajectory::recoverHome(uint32_t now) {
  const float home[5] = {0, 0, 0, 0, 0};
  move(home, Phase::RecoverHome, now);
}

bool ArmTrajectory::usesTimedPickup() const {
  return config_.pickupTimedCompletion && config_.boxOrientation && orientationLocked_;
}

uint32_t ArmTrajectory::closeCompletionMs() const {
  return config_.closingDurationMs + (usesTimedPickup() ? config_.pickupPostCloseMs : 0);
}

void ArmTrajectory::miss(const char *reason, const ArmFeedback &f, uint32_t now) {
  captureDiagnostic(ArmDiagnosticEvent::Missed, f, now, reason);
  const bool movedToPickup = phase_ != Phase::Plan && phase_ != Phase::Schedule &&
                             phase_ != Phase::Home && phase_ != Phase::Idle;
  missReason_ = reason;
  missedArrivalMs_ = arrivalDelayMs(now);
  orientationLocked_ = true;
  paused_ = pauseRequested_ = false; // No scheduled close may run after this point.
  if (closeIssued_ && uint32_t(now - closeMs_) < closeCompletionMs()) {
    phase_ = Phase::RecoverWait;
    stageMs_ = now;
  } else if (movedToPickup)
    move(above_, Phase::Recover, now);
  else
    recoverHome(now);
}

void ArmTrajectory::cancelPickup(const ArmFeedback &f, uint32_t now) {
  if (!healthy(f, now)) { fail("Nucleo feedback stale"); return; }
  if (phase_ == Phase::Recover || phase_ == Phase::RecoverWait || phase_ == Phase::RecoverHome)
    return;
  miss("OPERATOR_CANCELLED", f, now);
}
double ArmTrajectory::allowedLateness() const {
  return fmin(double(config_.maximumLateMs),
              double(config_.maximumCaptureErrorYmm) / fabs(double(velocity_)) * 1000.0);
}
void ArmTrajectory::plan(const float seed[5]) {
  const bool pickup = profilePlanActive_ ? planIndex_%3==1 : planIndex_%2;
  const bool sorting = profilePlanActive_ ? planIndex_%3==2 : planIndex_ && !pickup;
  diagnosticPlanStep_=sorting ? 2:pickup ? 1:0;
  const unsigned variant = profilePlanActive_ ? planIndex_/3 : planIndex_ ? (planIndex_-1)/2:0;
  const AutoConfig &c=profilePlanActive_ ? profilePlan_.config[variant]:config_;
  const float x=profilePlanActive_ ? profilePlan_.x[variant]:part_.x;
  float pose[6] = {x, 0, c.approachZ, c.approachRoll,c.approachPitch,c.approachYaw};
  if (pickup) {
    pose[2] = c.pickupZ;
    pose[3] = c.boxOrientation && variant == 0 ? c.approachRoll : c.pickupRoll;
    pose[4] = c.boxOrientation && variant == 0 ? c.approachPitch : c.pickupPitch;
    pose[5] = c.boxOrientation ? (variant ? 90 : 0) : c.pickupYaw;
  }
  if (sorting)
    memcpy(pose, destination_, sizeof(pose));
  float chosenSeed[5];
  memcpy(chosenSeed, seed, sizeof(chosenSeed));
  uint8_t fixedMask = 0;
  // The shared belt profile requests the full tested pickup orientation.
  // Other callers may choose a reduced task. Sorting relaxes orientation.
  IkTask task = sorting ? IkTask::PositionClosestOrientation : c.pickupTask;
  if(sorting && c.classificationFixedShoulders) {
    chosenSeed[0]=0;
    if(!sortingShoulderRoll(*params_,destination_,chosenSeed[1])) {
      reject("Classification shoulder-roll angle unavailable");return;
    }
    fixedMask=0x03;task=IkTask::FullPose;
  } else if (sorting && c.classificationPitchZero) {
    // On the classification ring, lock shoulder pitch at zero and let XYZ
    // determine roll/elbow/wrist. Tool orientation is deliberately unconstrained.
    chosenSeed[0] = 0;
    fixedMask = 1U << 0;
    task = IkTask::PositionOnly;
  }
  // Enforce q5 inside IK in every sorting mode. Changing it after solving can
  // invalidate XYZ, particularly with a future off-axis tool translation.
  if(sorting && c.holdPickupQ5ForSorting) {
    chosenSeed[4]=plannedPickup_[variant][4];fixedMask|=1U<<4;
  }
  memcpy(planPose_, pose, sizeof(planPose_));
  memcpy(planSeed_, chosenSeed, sizeof(planSeed_));
  ik_.begin(chosenSeed, pose, *params_, task, 150, .5f, .035f, fixedMask);
}
ArmPlanReport ArmTrajectory::planReport() const {
  ArmPlanReport report;
  memcpy(report.pose, planPose_, sizeof(report.pose));
  memcpy(report.seed, planSeed_, sizeof(report.seed));
  ik_.result(report.candidate);
  report.ik = ik_.report();
  return report;
}
bool ArmTrajectory::start(const Part &p, float v, const float destination[6], const AutoConfig &c,
                          const ArmFeedback &f, uint32_t now,const PickupProfilePlan *profiles) {
  if (busy() || outcome_ != PickOutcome::None)
    return false;
  config_ = c;
  classificationReportPending_=false;
  profilePlanActive_=profiles!=nullptr;profileReportPending_=false;
  activeProfile_=p.boxHeight>p.boxWidth ? 1:0;
  if(profiles){profilePlan_=*profiles;config_=profilePlan_.config[activeProfile_];}
  homeWaypointArrivalActive_ = settling_ = targetConfirmed_ = false;
  part_ = p;
  velocity_ = v;
  id_ = p.trackId;
  fault_ = "";
  faultStage_ = "";
  rejectionReason_ = "";
  paused_ = pauseRequested_ = false;
  closeIssued_ = orientationLocked_ = pickupArrived_ = homeKeepClosed_ = false;
  missReason_ = "";
  resetDiagnostics(f, now);
  orientationReportPending_ = false;
  selectedVariant_ = 0;
  if (!validArrivalConfiguration(c)) {
    fail("Invalid arrival tolerances");
    return false;
  }
  if (!healthy(f, now)) {
    fail("No fresh Nucleo feedback");
    return false;
  }
  if (!validPickupTiming(c)) {
    reject("Invalid pickup timing configuration");
    return false;
  }
  if (c.boxOrientation && (!p.boxWidth || !p.boxHeight)) {
    reject("No accepted bounding box for pickup orientation");
    return false;
  }
  if (!isfinite(v) || v >= -.1f || !isfinite(p.x) || !isfinite(p.y)) {
    reject("Invalid part coordinates or belt velocity");
    return false;
  }
  memcpy(destination_, destination, sizeof(destination_));
  memcpy(entrySeed_,f.q,sizeof(entrySeed_));
  lastSequence_ = f.sequence;
  planIndex_ = 0;
  diagnosticPlanStep_=0;
  phase_ = Phase::Plan;
  stageMs_ = now;
  admissionDelayMs_ = arrivalDelayMs(now);
  if(profilePlanActive_) {
    for(unsigned i=0;i<2;++i)
      profilePlan_.available[i]=profilePlan_.available[i] && validArrivalConfiguration(profilePlan_.config[i]) &&
          validPickupTiming(profilePlan_.config[i]) && isfinite(profilePlan_.config[i].pickupZ) &&
          isfinite(profilePlan_.x[i]);
    continueProfilePlan(f,now);
  } else plan(f.q);
  return true;
}
void ArmTrajectory::continueProfilePlan(const ArmFeedback &f,uint32_t now) {
  while(planIndex_<6 && !profilePlan_.available[planIndex_/3])planIndex_=uint8_t((planIndex_/3+1)*3);
  if(planIndex_<6) {
    const float home[5]={0,0,0,0,0};
    plan(planIndex_%3==1 ? plannedAbove_[planIndex_/3] : planIndex_%3==2 ? home:entrySeed_);
    return;
  }
  activeProfile_=part_.boxHeight>part_.boxWidth ? 1:0;
  if(!profilePlan_.available[activeProfile_]){reject("Selected box profile IK/workspace unavailable");return;}
  config_=profilePlan_.config[activeProfile_];part_.x=profilePlan_.x[activeProfile_];
  memcpy(above_,plannedAbove_[activeProfile_],sizeof(above_));
  phase_=Phase::Schedule;memcpy(target_,f.q,sizeof(target_));pauseAtBoundary(now);
  profileReportPending_=true;
}
bool ArmTrajectory::updateProfile(const ArmFeedback &f,uint32_t now) {
  if(!profilePlanActive_ || orientationLocked_ || paused_ ||
     (phase_!=Phase::Plan && phase_!=Phase::Schedule && phase_!=Phase::Above && phase_!=Phase::SafetyWait))return true;
  const unsigned requested=part_.boxHeight>part_.boxWidth ? 1:0;
  if(requested==activeProfile_)return true;
  if(phase_!=Phase::Plan && !profilePlan_.available[requested]) {
    miss("BOX_PROFILE_UNAVAILABLE",f,now);return false;
  }
  activeProfile_=requested;config_=profilePlan_.config[requested];part_.x=profilePlan_.x[requested];
  profileReportPending_=true;
  if(phase_==Phase::Above || phase_==Phase::SafetyWait) {
    if(arrivalDelayMs(now)<config_.descentLeadMs){miss("BOX_PROFILE_CHANGED_LATE",f,now);return false;}
    bool changed=false;
    for(unsigned i=0;i<5;++i)changed|=fabsf(plannedAbove_[requested][i]-above_[i])>.0001f;
    memcpy(above_,plannedAbove_[requested],sizeof(above_));
    if(changed && !move(above_,Phase::Above,now))return false;
  } else if(phase_==Phase::Schedule)memcpy(above_,plannedAbove_[requested],sizeof(above_));
  return true;
}
PickOutcome ArmTrajectory::takeOutcome(uint32_t &id) {
  id = id_;
  PickOutcome result = outcome_;
  outcome_ = PickOutcome::None;
  return result;
}
void ArmTrajectory::tick(const ArmFeedback &f, uint32_t now) {
  if (!busy()) return;
  tickGapMs_ = uint32_t(now - diagnosticTickMs_);
  if (tickGapMs_ > maximumTickGapMs_) maximumTickGapMs_ = tickGapMs_;
  diagnosticTickMs_ = now;
  if (f.sequence != diagnosticSequence_) {
    feedbackIntervalMs_ = uint32_t(f.timestampMs - diagnosticFeedbackMs_);
    diagnosticFeedbackMs_ = f.timestampMs;
    diagnosticSequence_ = f.sequence;
  }
  if (!healthy(f, now)) { fail("Nucleo feedback stale"); return; }
  if(!updateProfile(f,now))return;
  if (paused_) {
    // A completed timed close may pause even when contact keeps the
    // gripping pose outside tolerance. Feedback freshness is still mandatory.
    const bool timedGripPause = usesTimedPickup() && phase_ == Phase::Close;
    if (phase_ != Phase::Plan && phase_ != Phase::Schedule && !timedGripPause && !atTarget(f, true))
      fail("Paused pose lost");
    return;
  }
  const bool timedMotion = phase_ != Phase::Schedule && phase_ != Phase::SafetyWait &&
                           phase_ != Phase::Wait;
  if (timedMotion && now - stageMs_ > config_.motionTimeoutMs) {
    fail("Stage timeout"); return;
  }
  if (phase_ == Phase::Plan) {
    if (admissionDelayMs_ < config_.pickUpBudgetMs()) {
      reject("Insufficient arrival time for pickup budget"); return;
    }
    if (arrivalDelayMs(now) < double(config_.pickUpBudgetMs()) - config_.scheduleToleranceMs) {
      miss("PLAN_LATE", f, now); return;
    }
    IkState state = ik_.step(2);
    if(profilePlanActive_) {
      if(state!=IkState::Success && state!=IkState::Failed)return;
      const unsigned variant=planIndex_/3,step=planIndex_%3;
      if(state==IkState::Failed) {
        profilePlan_.available[variant]=false;planIndex_=uint8_t((variant+1)*3);
        continueProfilePlan(f,now);return;
      }
      if(step==0)ik_.result(plannedAbove_[variant]);
      else if(step==1)ik_.result(plannedPickup_[variant]);
      else {
        ik_.result(plannedDrop_[variant]);
        classificationIk_[variant]=ik_.report();
        float *drop=plannedDrop_[variant];const float *pickup=plannedPickup_[variant];
        if(profilePlan_.config[variant].holdPickupQ5ForSorting) {
          drop[4]=pickup[4];float actual[6];robot_fkine(drop,*params_,actual);
          const float error=hypotf(hypotf(actual[0]-destination_[0],actual[1]-destination_[1]),actual[2]-destination_[2]);
          if(!isfinite(error) || error>.5f)profilePlan_.available[variant]=false;
        }
        const float middle[5]={0,0,pickup[2],pickup[3],pickup[4]};
        for(unsigned i=0;i<5;++i) {
          const float rad=middle[i]*PI/180;
          if(!isfinite(middle[i]) || rad<params_->q_min[i] || rad>params_->q_max[i])profilePlan_.available[variant]=false;
        }
      }
      ++planIndex_;continueProfilePlan(f,now);return;
    }
    if (state == IkState::Failed) { reject("IK failed"); return; }
    if (state != IkState::Success) return;
    if (!planIndex_) {
      ik_.result(above_);
      ++planIndex_; plan(above_); return;
    }
    const unsigned variant = (planIndex_ - 1) / 2;
    if (planIndex_ % 2) {
      ik_.result(plannedPickup_[variant]);
      ++planIndex_;
      const float home[5] = {0, 0, 0, 0, 0};
      plan(home); return;
    }
    ik_.result(plannedDrop_[variant]);
    classificationIk_[variant]=ik_.report();
    float *drop = plannedDrop_[variant];
    const float *pickup = plannedPickup_[variant];
    if (config_.holdPickupQ5ForSorting) {
      drop[4] = pickup[4];
      float actual[6]; robot_fkine(drop, *params_, actual);
      const float error = hypotf(hypotf(actual[0] - destination_[0], actual[1] - destination_[1]),
                                 actual[2] - destination_[2]);
      if (!isfinite(error) || error > .5f) {
        reject("Classification XYZ unreachable with pickup q5 held"); return;
      }
    }
    // Validate the waypoint for BOTH orientations before approach starts.
    const float middle[5] = {0, 0, pickup[2], pickup[3], pickup[4]};
    for (int i=0; i<5; ++i) {
      const float radians = middle[i] * PI / 180;
      if (!isfinite(middle[i]) || radians < params_->q_min[i] || radians > params_->q_max[i]) {
        reject("Classification waypoint outside joint limits"); return;
      }
    }
    if (config_.boxOrientation && variant == 0) {
      ++planIndex_; plan(above_); return;
    }
    phase_ = Phase::Schedule;
    // Retain the feedback target while waiting at HOME (or caller's safe pose).
    memcpy(target_, f.q, sizeof(target_));
    pauseAtBoundary(now);
    return;
  }
  switch (phase_) {
  case Phase::Home:
    if (arrived(f, now) && (homeKeepClosed_ || now-stageMs_ >= config_.openingDurationMs) &&
        !pauseAtBoundary(now)) finish(PickOutcome::HomeReady);
    break;
  case Phase::Schedule: {
    if (pauseAtBoundary(now)) break;
    const double delay = arrivalDelayMs(now) - config_.pickUpBudgetMs();
    if (delay < -double(config_.scheduleToleranceMs)) { miss("APPROACH_COMMAND_LATE", f, now); break; }
    if (delay <= 0) move(above_, Phase::Above, now);
    break;
  }
  case Phase::Above:
    if (arrived(f, now)) {
      captureDiagnostic(ArmDiagnosticEvent::SafetyConfirmed, f, now);
      // Evaluate ONCE at confirmation. A later scheduled tick is a separate check.
      if (arrivalDelayMs(now) < config_.descentLeadMs) { miss("SAFETY_HEIGHT_LATE", f, now); break; }
      phase_ = Phase::SafetyWait;
      pauseAtBoundary(now);
    } else if (!pauseRequested_ && arrivalDelayMs(now) < config_.descentLeadMs)
      miss("SAFETY_HEIGHT_LATE", f, now);
    break;
  case Phase::SafetyWait: {
    if (!atTarget(f, true)) { miss("SAFETY_HEIGHT_LOST", f, now); break; }
    if (pauseAtBoundary(now)) break;
    const double delay = arrivalDelayMs(now) - config_.descentLeadMs;
    if (delay < -double(config_.scheduleToleranceMs)) { miss("DESCENT_COMMAND_LATE", f, now); break; }
    if (delay > 0) break;
    selectedVariant_ = config_.boxOrientation && part_.boxHeight > part_.boxWidth ? 1 : 0;
    orientationLocked_ = true;
    orientationReportPending_ = true;
    memcpy(pickup_, plannedPickup_[selectedVariant_], sizeof(pickup_));
    memcpy(drop_, plannedDrop_[selectedVariant_], sizeof(drop_));
    middle_[0] = middle_[1] = 0;
    middle_[2] = pickup_[2]; middle_[3] = pickup_[3]; middle_[4] = pickup_[4];
    if (move(pickup_, Phase::Down, now)) {
      classificationReportPending_=true;
      captureDiagnostic(ArmDiagnosticEvent::DescentCommand, f, now);
    }
    break;
  }
  case Phase::Down:
  case Phase::Wait:
  case Phase::Close: {
    const bool timedPickup = usesTimedPickup();
    if (!timedPickup && pickupArrived_ && !atTarget(f, true)) { miss("PICKUP_POSE_LOST", f, now); break; }
    const bool ready = arrived(f, now);
    if (!timedPickup && ready && !pickupArrived_)
      captureDiagnostic(ArmDiagnosticEvent::GrippingConfirmed, f, now);
    pickupArrived_ |= ready;
    const bool closeComplete = closeIssued_ && uint32_t(now-closeMs_) >= closeCompletionMs();
    // During descent, Y requests finish the commanded move. If closing already
    // started, finish that too; never start a new timed action while pausing.
    if (pauseRequested_ && (closeIssued_ ? closeComplete && (timedPickup || ready) : ready)) {
      phase_ = closeIssued_ ? Phase::Close : Phase::Wait;
      pauseAtBoundary(now); break;
    }
    if (!pauseRequested_) {
      // Legacy position-verified pickups require confirmed pose by Y=0. Timed pickups
      // deliberately allow contact-induced motion without this pose gate.
      if (!timedPickup && !pickupArrived_ && arrivalDelayMs(now) <= 0) {
        miss("GRIPPING_HEIGHT_LATE", f, now); break;
      }
      if (!closeIssued_ && closingDelay(now) < -allowedLateness()) {
        miss("CLOSE_COMMAND_LATE", f, now); break;
      }
      if (!closeIssued_ && closingDelay(now) <= 0) {
        if (!io_.gripper(config_.closeAngle)) { fail("Close command failed"); break; }
        closeIssued_ = true; closeMs_ = now; phase_ = Phase::Close;
        captureDiagnostic(ArmDiagnosticEvent::CloseCommand, f, now);
      }
    }
    if (!closeIssued_ && pickupArrived_) phase_ = Phase::Wait;
    if (closeComplete && (timedPickup || pickupArrived_) && !pauseAtBoundary(now)) {
      if (timedPickup)
        captureDiagnostic(ArmDiagnosticEvent::TimedPickupComplete, f, now);
      middle_[2] = f.q[2]; middle_[3] = f.q[3];
      move(middle_, Phase::Mid, now);
    }
    break;
  }
  case Phase::Mid:
    if (arrived(f, now) && !pauseAtBoundary(now)) move(drop_, Phase::Drop, now);
    break;
  case Phase::Drop:
    if (arrived(f, now) && !pauseAtBoundary(now)) {
      if (!io_.gripper(config_.openAngle)) { fail("Open command failed"); break; }
      closeIssued_ = false;
      phase_ = Phase::Open; stageMs_ = now;
    }
    break;
  case Phase::Open:
    if (!atTarget(f, true)) { fail("Classification pose lost while opening"); break; }
    if (now-stageMs_ >= config_.openingDurationMs && !pauseAtBoundary(now)) finish(PickOutcome::Sorted);
    break;
  case Phase::RecoverWait:
    if (uint32_t(now-closeMs_) >= closeCompletionMs()) move(above_, Phase::Recover, now);
    break;
  case Phase::Recover:
    if (arrived(f, now)) recoverHome(now);
    break;
  case Phase::RecoverHome:
    if (arrived(f, now)) finish(PickOutcome::Missed);
    break;
  default: break;
  }
}
