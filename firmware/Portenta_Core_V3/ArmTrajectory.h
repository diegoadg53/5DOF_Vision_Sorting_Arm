#pragma once
#include "BeltTracker.h"
#include "PickGeometryDefaults.h"

struct ArmFeedback {
  // A: telemetry order, in degrees: pitch, roll, j3, j4, j5.
  float q[5]{};
  uint32_t timestampMs = 0, sequence = 0;
  bool valid = false;
};
struct Destination {
  uint8_t binId = 0;
  bool enabled = false;
  // TCP x/y/z in mm, ZYX Euler roll/pitch/yaw in degrees.
  float pose[6]{};
};
struct ArrivalTolerance {
  // A: telemetry joint order, degrees: pitch, roll, j3, j4, j5.
  float jointToleranceDeg[5] = {1, 1, 1, 1, 1};
  float positionToleranceMm = 5;
  // Enter settling inside positionToleranceMm; reset only outside the sum.
  float positionHysteresisMm = 0;
};
struct AutoConfig {
  // Previous DH-origin Z targets converted at nominal roll=pitch=0.
  float pickupZ = -34, approachZ = 51;
  float heightReferenceXmm = PickMotion::heightReferenceXmm;
  float fittedContactZmm = PickMotion::fittedContactZmm;
  float pickupHeightAdjustmentMm = PickMotion::pickupHeightAdjustmentMm;
  float pickupRoll = 0, pickupPitch = 0, pickupYaw = 90;
  float approachRoll = 0, approachPitch = 0, approachYaw = 90;
  // Preplan yaw 0 and yaw 90; choose using the last accepted box at descent.
  // Yaw 0 uses approach roll/pitch; yaw 90 uses pickup roll/pitch.
  bool boxOrientation = false;
  IkTask pickupTask = IkTask::PositionPitchYaw;
  // Keep the pickup q5 angle during both sorting movements. Sorting
  // orientation is then relaxed; the modified target must still pass TCP XYZ.
  bool holdPickupQ5ForSorting = false;
  bool classificationPitchZero = false;
  // Ring sorting: fix shoulder pitch=0 and roll at the selected radial angle.
  // Solve XYZ and full TCP orientation using only q3/q4/q5.
  bool classificationFixedShoulders = false;
  // Must stay within the gripper limits reported by Nucleo.
  float openAngle = 0, closeAngle = 175;
  float jointToleranceDeg = 1, positionToleranceMm = 5;
  // Per-joint overrides in pitch/roll/j3/q4/q5 order; 0 inherits the scalar.
  float jointToleranceOverrideDeg[5]{};
  // Extra joint allowance ONLY after this target has fully settled/arrived.
  // Each new movement must first meet its original entry tolerance.
  float jointHoldHysteresisDeg[5]{};
  // Optional profile for HOME and the classification waypoint only.
  // This profile takes precedence over the scalar/per-joint joint limits.
  bool useHomeWaypointArrival = false;
  ArrivalTolerance homeWaypointArrival;
  float maximumCaptureErrorYmm = 5;
  // Measure on the robot: command-to-capture and command-to-completion.
  uint32_t closingLeadMs = 400;
  uint32_t closingDurationMs = 520;
  uint32_t openingDurationMs = 500;
  // Optional WIDE/TALL pickup completion by time alone after box/profile lock.
  // Safety height and subsequent movements still require position confirmation.
  bool pickupTimedCompletion = false;
  uint32_t pickupPostCloseMs = 100;
  // EDIT HERE: offsets from predicted arrival at fixed Y=0, in milliseconds.
  uint32_t descentLeadMs = 800;
  uint32_t pickUpMarginMs = 1600;
  // Loop jitter allowance for issuing scheduled approach/descent commands.
  // Does NOT relax the requirement to confirm safety height before descent.
  uint32_t scheduleToleranceMs = 25;
  uint32_t pickUpBudgetMs() const {
    return pickUpMarginMs + (descentLeadMs > closingLeadMs ? descentLeadMs : closingLeadMs);
  }
  uint32_t motionTimeoutMs = 10000;
  uint32_t settleMs = 250, minimumMoveMs = 300, feedbackTimeoutMs = 300, maximumLateMs = 50;
  float defaultDestination[6] = {0, 200, 251, 0, 0, 0};
  // Enable only after assigning measured TCP bin poses.
  Destination destinations[3]{{1, false, {0, 0, 0, 0, 0, 0}},
                              {2, false, {0, 0, 0, 0, 0, 0}},
                              {3, false, {0, 0, 0, 0, 0, 0}}};
};
bool validArrivalConfiguration(const AutoConfig &config);
bool validPickupTiming(const AutoConfig &config);
struct PickupProfilePlan {
  AutoConfig config[2];
  float x[2]{};
  bool available[2]={true,true};
};
enum class PickOutcome : uint8_t { None, HomeReady, Sorted, Missed, Rejected, Fault };
struct ArmCommands {
  bool (*joints)(const float *) = nullptr;
  bool (*gripper)(float) = nullptr;
  // Send mode + HOME position + open together in an ordered UART batch.
  bool (*home)(const float *, float) = nullptr;
};
struct ArmProgress {
  uint32_t elapsedMs = 0, feedbackAgeMs = 0, stableMs = 0;
  uint32_t tickGapMs = 0, maximumTickGapMs = 0, feedbackIntervalMs = 0;
  float target[5]{}, error[5]{}, positionErrorMm = NAN;
  float jointToleranceDeg[5]{};
  float jointEntryToleranceDeg[5]{}, jointHoldToleranceDeg[5]{};
  bool jointHoldActive = false;
  float positionEntryToleranceMm = 0, positionHoldToleranceMm = 0;
  bool feedbackValid = false, withinTarget = false, withinEntry = false;
  bool settlingStarted = false;
};
enum class ArmDiagnosticEvent : uint8_t {
  SafetyConfirmed, DescentCommand, CloseCommand, GrippingConfirmed, TimedPickupComplete, Missed
};
// Immutable event-time data. Recovery may replace target_/stageMs_ immediately
// after capture; diagnostic output must never read that replacement target.
struct ArmDiagnosticReport {
  ArmDiagnosticEvent event = ArmDiagnosticEvent::Missed;
  const char *reason = "", *stage = "", *targetName = "", *requirement = "";
  uint32_t trackId = 0;
  ArmProgress progress;
  double remainingArrivalMs = NAN, thresholdMs = 0, allowedLateMs = 0;
  bool hasDeadline = false, closingStarted = false;
  bool timedPickup = false, tallPickup = false;
  uint32_t postCloseMs = 0;
  uint32_t settleMs = 0, minimumMoveMs = 0, marginMs = 0, descentLeadMs = 0;
  uint32_t closingLeadMs = 0, closingDurationMs = 0, budgetMs = 0, closeElapsedMs = 0;
};
struct ArmPlanReport {
  float pose[6]{}, seed[5]{}, candidate[5]{};
  IkReport ik;
};
struct ClassificationPlanReport {
  uint32_t trackId=0;
  bool fixedShoulders=false,heldPickupQ5=false;
  float requested[6]{},planned[6]{},joints[5]{};
  IkReport ik;
};
class ArmTrajectory {
public:
  void bind(const RobotDHParams &p, ArmCommands io) {
    params_ = &p;
    io_ = io;
  }
  bool prepare(const AutoConfig &c, const ArmFeedback &f, uint32_t now, bool keepClosed = false);
  bool start(const Part &p, float velocity, const float destination[6], const AutoConfig &c,
             const ArmFeedback &f, uint32_t now, const PickupProfilePlan *profiles=nullptr);
  void tick(const ArmFeedback &f, uint32_t now);
  void updateObservation(const Part &p);
  bool takeClassificationPlan(ClassificationPlanReport &report);
  const AutoConfig &configuration()const{return config_;}
  bool takeProfileReport(){bool p=profileReportPending_;profileReportPending_=false;return p;}
  unsigned activeProfile()const{return activeProfile_;}
  // Controlled cancellation: suppress timers, lift if needed, HOME, no release.
  void cancelPickup(const ArmFeedback &f, uint32_t now);
  uint32_t trackId() const { return id_; }
  bool clawMayHoldPart() const { return closeIssued_; }
  bool orientationLocked() const { return orientationLocked_; }
  float selectedPickupYaw() const { return selectedVariant_ ? 90 : 0; }
  const char *missReason() const { return missReason_; }
  double missedArrivalMs() const { return missedArrivalMs_; }
  bool takeDiagnostic(ArmDiagnosticReport &report);
  bool takeOrientationReport() { bool value = orientationReportPending_; orientationReportPending_ = false; return value; }
  double arrivalDelayMs(uint32_t now) const;
  // Finish the active action, then hold before issuing the next action.
  // Freeze the assigned part's clock from the pause request until resume.
  void requestPause(uint32_t now);
  bool resume(uint32_t now);
  bool paused() const {
    return paused_;
  }
  bool pauseRequested() const {
    return pauseRequested_;
  }
  const char *phaseName() const;
  const char *faultStage() const {
    return faultStage_;
  }
  ArmProgress progress(const ArmFeedback &f, uint32_t now) const;
  ArmPlanReport planReport() const;
  const char *planTargetName() const {
    return diagnosticPlanStep_==0 ? "APPROACH":diagnosticPlanStep_==1 ? "PICKUP":"CLASSIFICATION";
  }
  const char *rejectionReason() const {
    return rejectionReason_;
  }
  void cancel() {
    phase_ = Phase::Idle;
    outcome_ = PickOutcome::None;
    id_ = 0;
    paused_ = pauseRequested_ = false;
    settling_ = homeWaypointArrivalActive_ = false;
    diagnosticRead_ = diagnosticCount_ = 0;
    classificationReportPending_=false;
  }
  bool busy() const {
    return phase_ != Phase::Idle;
  }
  PickOutcome takeOutcome(uint32_t &id);
  const char *fault() const {
    return fault_;
  }

private:
  enum class Phase : uint8_t {
    Idle,
    Plan,
    Schedule,
    Home,
    Above,
    SafetyWait,
    Down,
    Wait,
    Close,
    Mid,
    Drop,
    Open,
    RecoverWait,
    Recover,
    RecoverHome
  };
  Phase phase_ = Phase::Idle;
  PickOutcome outcome_ = PickOutcome::None;
  AutoConfig config_;
  PickupProfilePlan profilePlan_;
  bool profilePlanActive_=false,profileReportPending_=false;
  unsigned activeProfile_=0;
  float plannedAbove_[2][5]{},entrySeed_[5]{};
  Part part_;
  const RobotDHParams *params_ = nullptr;
  ArmCommands io_;
  IkStepper ik_;
  float velocity_ = 0, destination_[6]{}, above_[5]{}, pickup_[5]{}, drop_[5]{}, middle_[5]{},
        target_[5]{};
  float plannedPickup_[2][5]{}, plannedDrop_[2][5]{};
  IkReport classificationIk_[2];
  bool classificationReportPending_=false;
  uint32_t id_ = 0, stageMs_ = 0, settledSince_ = 0, lastSequence_ = 0;
  double admissionDelayMs_ = 0, missedArrivalMs_ = 0;
  bool closeIssued_ = false, orientationLocked_ = false, pickupArrived_ = false;
  bool homeKeepClosed_ = false;
  bool orientationReportPending_ = false;
  // Bounded event queue, drained outside control decisions. A cycle produces
  // at most five events; do not allocate memory or print from tick().
  ArmDiagnosticReport diagnostics_[8];
  uint8_t diagnosticRead_ = 0, diagnosticCount_ = 0;
  uint32_t diagnosticTickMs_ = 0, diagnosticFeedbackMs_ = 0, diagnosticSequence_ = 0;
  uint32_t tickGapMs_ = 0, maximumTickGapMs_ = 0, feedbackIntervalMs_ = 0;
  uint8_t selectedVariant_ = 0;
  uint32_t closeMs_ = 0;
  const char *missReason_ = "";
  bool settling_ = false;
  bool homeWaypointArrivalActive_ = false;
  bool paused_ = false, pauseRequested_ = false;
  uint32_t pauseRequestedMs_ = 0, pauseBoundaryMs_ = 0;
  uint8_t planIndex_ = 0;
  uint8_t diagnosticPlanStep_=0;
  const char *fault_ = "";
  const char *faultStage_ = "";
  const char *rejectionReason_ = "";
  float planPose_[6]{}, planSeed_[5]{};
  bool healthy(const ArmFeedback &f, uint32_t now) const;
  bool targetConfirmed_ = false;
  float jointTolerance(int joint, bool holding = false) const;
  float positionTolerance(bool holding) const;
  bool atTarget(const ArmFeedback &f, bool holding = false) const;
  bool arrived(const ArmFeedback &f, uint32_t now);
  bool move(const float q[5], Phase phase, uint32_t now);
  void continueProfilePlan(const ArmFeedback &f,uint32_t now);
  bool updateProfile(const ArmFeedback &f,uint32_t now);
  bool usesTimedPickup() const;
  uint32_t closeCompletionMs() const;
  bool pauseAtBoundary(uint32_t now);
  void finish(PickOutcome result) {
    outcome_ = result;
    phase_ = Phase::Idle;
    paused_ = pauseRequested_ = false;
  }
  void fail(const char *reason) {
    faultStage_ = phaseName();
    fault_ = reason;
    finish(PickOutcome::Fault);
  }
  void reject(const char *reason) {
    rejectionReason_ = reason;
    finish(PickOutcome::Rejected);
  }
  void plan(const float seed[5]);
  void miss(const char *reason, const ArmFeedback &feedback, uint32_t now);
  void resetDiagnostics(const ArmFeedback &feedback, uint32_t now);
  void captureDiagnostic(ArmDiagnosticEvent event, const ArmFeedback &feedback,
                         uint32_t now, const char *reason = "");
  void recoverHome(uint32_t now);
  double closingDelay(uint32_t now) const;
  double allowedLateness() const;
};
