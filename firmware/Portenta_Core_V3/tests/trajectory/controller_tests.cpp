#include "ArmTrajectory.h"
#include "ArmTrajectoryDiagnostics.h"
#include "SettingsRecord.h"
#include "PortentaStorageLayout.h"
#include "TrajectoryTestConfig.h"
#include "AutoPickSelection.h"
#include "JointCalibrationWizard.h"
#include "JointMotionTest.h"
#include "CameraAlgorithmSelection.h"
#include "BeltCalibrationSession.h"
#include "../../../ESP32_V3/DiagnosticQueue.h"
#include "../../../ESP32_V3/TagVision.h"
#include <string.h>
#include <stdio.h>
#include <new>

// Actual main-sketch HOME adapter, with UART transport/clock simulated.
static RobotDHParams params;
static AutoConfig autoConfig;
static ArmFeedback armFeedback;
static bool limits_received;
static float LIMIT_GRIPPER_MIN = 0, LIMIT_GRIPPER_MAX = 180;
static uint32_t adapterTime;
static uint32_t millis() {
  return adapterTime;
}
struct MockUart {
  char batch[256]{};
  uint32_t writes = 0;
  bool shortWrite = false;
  size_t print(const char *text) {
    return write(reinterpret_cast<const uint8_t *>(text), strlen(text));
  }
  size_t write(const uint8_t *b, size_t n) {
    ++writes;
    if (n >= sizeof(batch))
      return 0;
    memcpy(batch, b, n);
    batch[n] = '\0';
    return shortWrite ? n - 1 : n;
  }
} Serial1;
static Print Serial;
static JointCalibrationWizard jointWizard;
static JointMotionTest jointMotionTest;
#include "AppHomeFunctions.inc"

// Compiled with the real ArmTrajectory and kinematics implementations.
// Only outgoing commands and incoming physical feedback are simulated.
struct TestResult {
  uint32_t checks, failures, firstFailure;
};
extern "C" {
volatile TestResult test_result = {};
char test_context[192] = {};
char runtime_settings_debug[1024] = {};
uint32_t failure_codes[128] = {};
}

static float commanded[5];
static uint32_t positionCommands, gripperCommands, ioTime, closeTime;
static bool sawClose;

static void check(bool condition, uint32_t code) {
  ++test_result.checks;
  if (!condition) {
    if (test_result.failures < 128) failure_codes[test_result.failures] = code;
    ++test_result.failures;
    if (!test_result.firstFailure)
      test_result.firstFailure = code;
  }
}
static bool sendJoints(const float *q) {
  memcpy(commanded, q, sizeof(commanded));
  ++positionCommands;
  return true;
}
// Exercise the configured camera IDs, rather than assuming IDs equal bin numbers.
static uint8_t partForBin(uint8_t bin) {
  for (const auto &route : Classification::partRoutes)
    if (route.binId == bin) return route.partId;
  return 0;
}
static bool sendGripper(float angle) {
  ++gripperCommands;
  if (angle == AutoConfig{}.closeAngle) {
    sawClose = true;
    closeTime = ioTime;
  }
  return true;
}

struct MemorySettingsFlash {
  uint8_t bytes[8192]; uint32_t bankSize=4096;
  const char *error=nullptr;
  unsigned operation=0,failAt=0,erasures=0; int partialBytes=-1;
  void reset() {memset(bytes,0xff,sizeof(bytes));bankSize=4096;error=nullptr;operation=failAt=erasures=0;partialBytes=-1;}
  bool begin(){return true;}
  bool read(uint32_t a,void *p,unsigned n) {
    if(a>sizeof(bytes) || n>sizeof(bytes)-a)return false;memcpy(p,bytes+a,n);return true;
  }
  bool erase(uint32_t a,unsigned n) {
    if(++operation==failAt)return false;
    if(a%bankSize || n!=bankSize || a>=sizeof(bytes))return false;
    memset(bytes+a,0xff,n);++erasures;return true;
  }
  bool program(uint32_t a,const void *p,unsigned n) {
    const uint8_t *b=static_cast<const uint8_t *>(p);
    if(a>sizeof(bytes) || n>sizeof(bytes)-a)return false;
    if(++operation==failAt) {
      if(partialBytes<0)return false;
      n=n<unsigned(partialBytes) ? n:unsigned(partialBytes);
      for(unsigned i=0;i<n;++i)bytes[a+i]&=b[i];return false;
    }
    for(unsigned i=0;i<n;++i)bytes[a+i]&=b[i];return true;
  }
  bool sync(){return ++operation!=failAt;}
};
static MemorySettingsFlash settingsFlash;
static SettingsJournal<MemorySettingsFlash> settingsStore(settingsFlash);
static RuntimeTuning activeTuning,tuningDraft;
static bool tuningMenuRequested;
static unsigned tuningRow;
static int8_t tuningLastX,tuningLastY;
static uint32_t tuningRepeatMs;
static bool autoTuningLaunchPending,pickupSaveRequested,pickupEditorLocked,pickupEditorNeedsNeutral;
static unsigned pickupTuningProfile,pickupTuningRow;
static int8_t pickupLastX,pickupLastY;
static uint32_t pickupRepeatMs;
bool applyTuningDraft();
bool servicePickupTuning(uint32_t,uint16_t);
void savePickupTuning();
void printPickupTuningRow();

// Actual sketch AUTO coordinator and supervisor; only transport/clock are fake.
#include "AppAutoTypes.inc"
static ArmTrajectory arm;
// Older regression fixtures deliberately equate pixels with world mm, including
// Y>480. Give only those synthetic fixtures a larger rectangle. ROI-specific
// tests below exercise the actual 640x480 production defaults.
static constexpr CameraTracking::PixelRectangle fixtureVelocityPixels = {0, 0, 1000, 1000};
static BeltTracker mainTracker(CameraTracking::partXOffsetMm, fixtureVelocityPixels);
static calibH camera_calib;
static State currentState = AUTO_MODE, lastState = MODE_SELECTOR;
static State tuningReturnState=CALIBRATION_MENU;
static AutoRun autoRun = AutoRun::Stopped;
static uint8_t menu_option0 = 1, menu_option1 = 0, menu_option2 = 0;
static constexpr uint8_t APRIL_TAG_ALGO_ID = 16, CUSTOM_MODEL1_ID = 131, CUSTOM_MODEL2_ID = 129;
static bool requestedTagsAlgo, requestedAlgoVelocity, algoSwitched, invalidAlgo, switchAcked;
static uint32_t taskStartedMs;
static CameraAlgorithmSelection cameraAlgorithm;
static BeltCalibrator beltCalibration;
static BeltCalibrationSession beltCalibrationSession;
static CalPhase calPhase;
static uint32_t lastVelocityCalibrationLogMs;
static uint32_t sortedParts, rejectedParts, ignoredParts, outsidePickupParts;
static int auto_parts_missed;
static uint32_t visionTimestampMs, lastArmProgressMs;
static uint8_t totalObjects;
static objectVision latestReceived[10];
static bool homographyDone, velocityCalibrated, cameraOffline, communicationError;
static bool cameraCancelRequested;
static bool streamStarted, stopRequested, exitRequested, newLatest;
static bool entered_ES, leave_ES, holdingResetCombo;
static bool udpControllerStable, controllerTimedOutThisLoop, controllerEmergencyReceived;
static struct { uint16_t buttons = 0; int8_t jsLx=0,jsLy=0; } currentControlCMD;
static criticalMessage pendingESPmsg;
static bool messageAck;
static const char *autoLastStage = "";
static uint8_t fakeAckType, lastCriticalType;
static uint32_t criticalCommands;
static uint8_t lastCriticalParam;
static void sendCriticalMessage(typeMsg type, uint8_t param = 0) {
  lastCriticalType = type;
  lastCriticalParam = param;
  if (type == CMD_SWITCH_ALGO) cameraAlgorithm.begin(param);
  ++criticalCommands;
  pendingESPmsg.pending = true;
}
static bool takeAck(typeMsg type) {
  if (fakeAckType != type)
    return false;
  fakeAckType = 0;
  pendingESPmsg.pending = false;
  return true;
}
#include "AppAutoFunctions.inc"

struct Fixture {
  RobotDHParams params;
  AutoConfig config;
  ArmTrajectory arm;
  ArmFeedback feedback;
  uint32_t now, startTime;

  explicit Fixture(uint32_t time = 1000)
      : params(robot_get_dh_params()), now(time), startTime(time) {
    memset(commanded, 0, sizeof(commanded));
    positionCommands = gripperCommands = closeTime = 0;
    sawClose = false;
    ioTime = now;
    config.pickUpMarginMs = 1000;
    config.descentLeadMs = 600; // Fixed regression fixture; production tuning remains editable.
    config.closingLeadMs = 500; // Fixed reference for timing/pause assertions.
    config.settleMs = config.minimumMoveMs = 20;
    config.openingDurationMs = 30; config.closingDurationMs = 520;
    feedback.valid = true;
    feedback.timestampMs = now;
    feedback.sequence = 1;
    arm.bind(params, {sendJoints, sendGripper});
  }
  void tick(uint32_t elapsed = 10, bool followCommand = true, bool fresh = true) {
    now += elapsed;
    ioTime = now;
    if (fresh) {
      feedback.timestampMs = now;
      ++feedback.sequence;
    }
    if (followCommand)
      memcpy(feedback.q, commanded, sizeof(commanded));
    arm.tick(feedback, now);
  }
  bool stage(const char *name) const {
    return strcmp(arm.phaseName(), name) == 0;
  }
  bool until(const char *name) {
    for (int i = 0; i < 1000; ++i) {
      if (stage(name))
        return true;
      if (!arm.busy())
        return false;
      tick();
    }
    return false;
  }
  bool pick(float x = 332) {
    Part p;
    p.trackId = 1;
    p.x = x;
    p.y = 400; p.boxWidth = 20; p.boxHeight = 40;
    p.stateMs = now;
    startTime = now;
    return arm.start(p, -100, config.defaultDestination, config, feedback, now);
  }
  PickOutcome finish() {
    for (int i = 0; arm.busy() && i < 1200; ++i)
      tick();
    uint32_t id;
    return arm.takeOutcome(id);
  }
};

static void normalRun() {
  Fixture f;
  check(f.pick(), 101);
  check(f.finish() == PickOutcome::Sorted, 102);
  check(positionCommands == 4 && gripperCommands == 2, 103);
  check(sawClose && uint32_t(closeTime - f.startTime) >= 3500 &&
            uint32_t(closeTime - f.startTime) <= 3520,
        104);
}

static void pauseMovingAndFreezeTime() {
  Fixture f;
  check(f.pick() && f.until("APPROACH"), 201);
  uint32_t requested = f.now;
  f.arm.requestPause(f.now);
  uint32_t commandsBefore = positionCommands;
  f.tick(200, false);
  check(!f.arm.paused() && f.arm.pauseRequested(), 202);
  check(f.stage("APPROACH") && positionCommands == commandsBefore, 203);
  for (int i = 0; !f.arm.paused() && i < 20; ++i)
    f.tick();
  check(f.arm.paused() && f.stage("SAFETY HEIGHT WAIT"), 204);
  f.tick(15000);
  check(f.arm.paused() && positionCommands == commandsBefore, 205);
  uint32_t frozen = f.now - requested;
  check(f.arm.resume(f.now), 206);
  f.tick();
  check(f.until("DESCEND") && positionCommands == commandsBefore + 1, 207);
  check(f.finish() == PickOutcome::Sorted, 208);
  check(uint32_t(closeTime - f.startTime - frozen) >= 3500 &&
            uint32_t(closeTime - f.startTime - frozen) <= 3520,
        209);
}

static void pauseCountdown() {
  Fixture f;
  check(f.pick() && f.until("WAIT FOR PART"), 301);
  uint32_t requested = f.now;
  f.arm.requestPause(f.now);
  f.tick();
  check(f.arm.paused() && !sawClose, 302);
  f.tick(20000);
  check(f.arm.paused() && !sawClose && gripperCommands == 0, 303);
  uint32_t frozen = f.now - requested;
  check(f.arm.resume(f.now), 304);
  check(f.finish() == PickOutcome::Sorted, 305);
  check(uint32_t(closeTime - f.startTime - frozen) >= 3500 &&
            uint32_t(closeTime - f.startTime - frozen) <= 3520,
        306);
}

static void pauseClosing() {
  Fixture f;
  check(f.pick() && f.until("CLOSE"), 401);
  uint32_t before = positionCommands;
  f.arm.requestPause(f.now);
  f.tick(10);
  check(!f.arm.paused() && positionCommands == before, 402);
  for (int i = 0; !f.arm.paused() && i < 100; ++i)
    f.tick();
  check(f.arm.paused() && f.stage("CLOSE") && positionCommands == before, 403);
  f.tick(15000);
  check(f.arm.resume(f.now), 404);
  f.tick();
  check(f.stage("CLASSIFICATION WAYPOINT") && positionCommands == before + 1, 405);
  check(f.finish() == PickOutcome::Sorted, 406);
}

static void pausedFeedbackStillRequired() {
  Fixture f;
  check(f.pick() && f.until("WAIT FOR PART"), 501);
  f.arm.requestPause(f.now);
  f.tick();
  check(f.arm.paused(), 502);
  uint32_t before = positionCommands;
  f.tick(301, true, false);
  uint32_t id;
  check(f.arm.takeOutcome(id) == PickOutcome::Fault, 503);
  check(!f.arm.busy() && !f.arm.paused() && positionCommands == before, 504);
}

static void cancelPausedAndHome() {
  Fixture f;
  check(f.pick() && f.until("WAIT FOR PART"), 601);
  f.arm.requestPause(f.now);
  f.tick();
  check(f.arm.paused(), 602);
  f.arm.cancel();
  check(!f.arm.busy() && !f.arm.paused() && !f.arm.pauseRequested(), 603);
  ioTime = f.now;
  check(f.arm.prepare(f.config, f.feedback, f.now), 604);
  check(f.stage("HOME"), 605);
  for (float value : commanded)
    check(value == 0, 606);
  check(f.finish() == PickOutcome::HomeReady, 607);
}

static void homePauseAndClockWrap() {
  Fixture f(0xfffffff0);
  check(f.arm.prepare(f.config, f.feedback, f.now), 701);
  f.arm.requestPause(f.now);
  for (int i = 0; !f.arm.paused() && i < 10; ++i)
    f.tick();
  check(f.arm.paused() && f.stage("HOME"), 702);
  f.tick(15000);
  check(f.arm.resume(f.now), 703);
  check(f.finish() == PickOutcome::HomeReady, 704);
}

static void conservativeAdmissionBudgetIsRespected() {
  Fixture f;
  f.config.pickUpMarginMs = 5000;
  check(f.pick(), 801);
  f.tick();
  uint32_t id;
  check(f.arm.takeOutcome(id) == PickOutcome::Rejected, 802);
  check(positionCommands == 0 && gripperCommands == 0, 803);
}

static void lostPausedPose() {
  Fixture f;
  check(f.pick() && f.until("WAIT FOR PART"), 901);
  f.arm.requestPause(f.now);
  f.tick();
  f.feedback.q[0] += 10;
  f.tick(10, false);
  uint32_t id;
  check(f.arm.takeOutcome(id) == PickOutcome::Fault, 902);
}

static void pausedPartClockAcrossWrap() {
  Fixture f(0xfffffca0);
  check(f.pick() && f.until("WAIT FOR PART"), 1001);
  uint32_t requested = f.now;
  f.arm.requestPause(f.now);
  f.tick();
  check(f.arm.paused(), 1002);
  f.tick(20000);
  uint32_t frozen = f.now - requested;
  check(f.arm.resume(f.now), 1003);
  check(f.finish() == PickOutcome::Sorted, 1004);
  check(uint32_t(closeTime - f.startTime - frozen) >= 3500 &&
            uint32_t(closeTime - f.startTime - frozen) <= 3520,
        1005);
}

static void homeCommandBatch() {
  Fixture f;
  params = f.params;
  armFeedback = f.feedback;
  limits_received = true;
  adapterTime = f.now;
  Serial1.writes = 0;
  Serial1.shortWrite = false;
  f.arm.bind(f.params, {sendJoints, sendGripper, sendAutoHome});
  check(f.arm.prepare(f.config, f.feedback, f.now), 1101);
  check(Serial1.writes == 1 && positionCommands == 0 && gripperCommands == 0, 1102);
  check(strcmp(Serial1.batch, "S:AUTO\nP:0.00,0.00,0.00,0.00,0.00\nG:0.00\n") == 0, 1103);
  check(f.finish() == PickOutcome::HomeReady, 1104);

  // A partial UART write must fault instead of pretending HOME was sent.
  Fixture failed;
  params = failed.params;
  armFeedback = failed.feedback;
  adapterTime = failed.now;
  Serial1.shortWrite = true;
  failed.arm.bind(failed.params, {sendJoints, sendGripper, sendAutoHome});
  check(!failed.arm.prepare(failed.config, failed.feedback, failed.now), 1105);
  uint32_t id;
  check(failed.arm.takeOutcome(id) == PickOutcome::Fault &&
            strcmp(failed.arm.fault(), "HOME command failed") == 0 &&
            strcmp(failed.arm.faultStage(), "HOME") == 0,
        1106);
  Serial1.shortWrite = false;

  // Internal limit checks still apply even with the combined HOME callback.
  Fixture limited;
  limited.params.q_min[0] = .1f;
  limited.arm.bind(limited.params, {sendJoints, sendGripper, sendAutoHome});
  uint32_t before = Serial1.writes;
  check(!limited.arm.prepare(limited.config, limited.feedback, limited.now), 1107);
  check(Serial1.writes == before && limited.arm.takeOutcome(id) == PickOutcome::Fault, 1108);

  // The real adapter independently rejects stale feedback without transmitting.
  adapterTime += autoConfig.feedbackTimeoutMs + 1;
  float home[5]{};
  check(!sendAutoHome(home, 0) && Serial1.writes == before, 1109);
}

static void homeTimeoutDiagnostics() {
  Fixture f;
  f.feedback.q[2] = 5;
  check(f.arm.prepare(f.config, f.feedback, f.now), 1201);
  f.tick(50, false);
  ArmProgress p = f.arm.progress(f.feedback, f.now);
  check(p.feedbackValid && !p.withinTarget && p.feedbackAgeMs == 0 && p.elapsedMs == 50, 1202);
  check(p.target[2] == 0 && p.error[2] == 5 && p.positionErrorMm > 5 && p.stableMs == 0, 1203);
  f.tick(f.config.motionTimeoutMs, false);
  uint32_t id;
  check(f.arm.takeOutcome(id) == PickOutcome::Fault && !f.arm.busy(), 1204);
  check(strcmp(f.arm.faultStage(), "HOME") == 0 && strcmp(f.arm.fault(), "Stage timeout") == 0,
        1205);
  p = f.arm.progress(f.feedback, f.now);
  check(p.feedbackValid && p.target[2] == 0 && p.error[2] == 5 && p.elapsedMs == 10050, 1206);
  check(positionCommands == 1 && gripperCommands == 1 && !sawClose, 1207);
}

static void recoveryAfterSlowApproach() {
  Fixture f;
  check(f.pick() && f.until("APPROACH"), 1301);
  // The robot has not reached approach when the virtual close deadline passes.
  f.tick(3600, false);
  check(f.stage("MISSED-PICK RECOVERY") && positionCommands == 2 && !sawClose, 1302);
  check(f.finish() == PickOutcome::Missed && !sawClose && gripperCommands == 0, 1303);

  Fixture stalled;
  check(stalled.pick() && stalled.until("APPROACH"), 1304);
  stalled.tick(3600, false);
  stalled.tick(stalled.config.motionTimeoutMs + 1, false);
  uint32_t id;
  check(stalled.arm.takeOutcome(id) == PickOutcome::Fault &&
            strcmp(stalled.arm.faultStage(), "MISSED-PICK RECOVERY") == 0,
        1305);
  ArmProgress p = stalled.arm.progress(stalled.feedback, stalled.now);
  check(p.feedbackValid && !p.withinTarget && p.positionErrorMm > 5, 1306);
  check(!sawClose && gripperCommands == 0, 1307);
}

static void jointToleranceDoesNotGuaranteeTcpTolerance() {
  Fixture f;
  f.feedback.q[0] = .9f;
  check(f.arm.prepare(f.config, f.feedback, f.now), 1401);
  f.tick(50, false);
  ArmProgress p = f.arm.progress(f.feedback, f.now);
  check(fabsf(p.error[0]) < f.config.jointToleranceDeg, 1402);
  check(p.positionErrorMm > f.config.positionToleranceMm && !p.withinTarget && p.stableMs == 0,
        1403);
  f.tick(f.config.motionTimeoutMs, false);
  uint32_t id;
  check(f.arm.takeOutcome(id) == PickOutcome::Fault, 1404);
}

static void quantizedReachablePoses() {
  RobotDHParams p = robot_get_dh_params();
  // Cases 32 and 72 previously failed after two evaluations: the floating
  // solution passed, then the rounded command barely exceeded 0.5 mm.
  for (int s = 0; s < 100; ++s) {
    float goal[5], seed[5], pose[6], solution[5], actual[6];
    for (int i = 0; i < 5; ++i) {
      goal[i] = ((s * (i * 7 + 3) + i * 11) % 31 - 15) * 1.7f;
      seed[i] = goal[i] + ((s + i * 3) % 7 - 3) * .11f;
    }
    robot_fkine(goal, p, pose);
    IkReport report;
    bool ok =
        robot_ikine(seed, pose, 150, .5f, .035f, p, solution, IkTask::PositionPitchYaw, &report);
    check(ok && report.state == IkState::Success, 1501 + s);
    if (!ok)
      continue;
    robot_fkine(solution, p, actual);
    check(hypotf(hypotf(actual[0] - pose[0], actual[1] - pose[1]), actual[2] - pose[2]) <= .5f,
          1601 + s);
    for (int i = 0; i < 5; ++i)
      check(fabsf(solution[i] * 100 - roundf(solution[i] * 100)) < .001f &&
                solution[i] * PI / 180 >= p.q_min[i] && solution[i] * PI / 180 <= p.q_max[i],
            1701 + s);
  }
}

static void manualProjectionAndTransactionalTarget() {
  RobotDHParams p = robot_get_dh_params();
  float home[5]{}, pose[6], output[5];
  robot_fkine(home, p, pose);
  pose[3] += 8;
  IkReport strict, relaxed;
  check(!robot_ikine(home, pose, 150, .5f, .035f, p, output, IkTask::FullPose, &strict), 1801);
  check(strict.failure == IkFailure::IterationLimit && strict.positionErrorMm <= .5f, 1802);
  check(robot_ikine(home, pose, 150, .5f, .035f, p, output, IkTask::PositionClosestOrientation,
                    &relaxed),
        1803);
  check(relaxed.controlledRotationErrorRad <= .035f && relaxed.fullRotationErrorRad > 7 * PI / 180,
        1804);
  const float initial[2][5] = {{0, 0, 0, 0, 0}, {20, 30, -15, -50, -20}};
  for (int s = 0; s < 2; ++s)
    for (int axis = 0; axis < 6; ++axis)
      for (int sign = -1; sign <= 1; sign += 2) {
        float seed[5], target[6], expected[6], increment[6]{}, actual[6];
        memcpy(seed, initial[s], sizeof(seed));
        robot_fkine(seed, p, target);
        memcpy(expected, target, sizeof(expected));
        increment[axis] = sign * (axis < 3 ? 2.5f : 4.f);
        expected[axis] += increment[axis];
        bool ok = robot_ikine_increment(seed, target, increment, 150, .5f, .035f, p, relaxed);
        check(ok, 1810 + s * 20 + axis);
        if (!ok)
          continue;
        check(memcmp(target, expected, sizeof(target)) == 0, 1850 + s * 20 + axis);
        robot_fkine(seed, p, actual);
        check(hypotf(hypotf(actual[0] - target[0], actual[1] - target[1]), actual[2] - target[2]) <=
                      .5f &&
                  relaxed.controlledRotationErrorRad <= .035f,
              1890 + s * 20 + axis);
      }
  float seed[5]{}, target[6], before[6], oldSeed[5]{};
  robot_fkine(seed, p, target);
  memcpy(before, target, sizeof(before));
  float impossible[6] = {5000, 0, 0, 0, 0, 0};
  check(!robot_ikine_increment(seed, target, impossible, 50, .5f, .035f, p, relaxed), 1930);
  check(memcmp(target, before, sizeof(before)) == 0 && memcmp(seed, oldSeed, sizeof(seed)) == 0,
        1931);
  float backToReachable[6] = {2.5f, 0, 0, 0, 0, 0};
  check(robot_ikine_increment(seed, target, backToReachable, 150, .5f, .035f, p, relaxed), 1932);
}

static void realisticHomeAndRejectionDiagnostics() {
  Fixture f;
  const float measuredHome[5] = {.33f, .03f, -.14f, .02f, 0};
  memcpy(f.feedback.q, measuredHome, sizeof(measuredHome));
  check(f.pick() && f.until("APPROACH"), 2001);
  check(f.finish() == PickOutcome::Sorted, 2002);
  check(f.arm.planReport().ik.task == IkTask::PositionClosestOrientation, 2003);

  Fixture deadline;
  deadline.config.pickUpMarginMs = 5000;
  check(deadline.pick(), 2004);
  deadline.tick();
  uint32_t id;
  check(deadline.arm.takeOutcome(id) == PickOutcome::Rejected &&
            strcmp(deadline.arm.rejectionReason(),
                   "Insufficient arrival time for pickup budget") == 0,
        2005);

  Fixture invalid;
  invalid.config.pickupPitch = 89;
  check(invalid.pick(), 2006);
  for(int i=0;invalid.arm.busy() && i<200;++i) invalid.tick();
  check(invalid.arm.takeOutcome(id) == PickOutcome::Rejected &&
            strcmp(invalid.arm.planTargetName(), "PICKUP") == 0 &&
            invalid.arm.planReport().ik.failure == IkFailure::InvalidInput && positionCommands == 0,
        2007);

  Fixture unreachableBin;
  unreachableBin.config.defaultDestination[0] = 5000;
  check(unreachableBin.pick(), 2008);
  check(unreachableBin.finish() == PickOutcome::Rejected &&
            strcmp(unreachableBin.arm.rejectionReason(), "IK failed") == 0 &&
            strcmp(unreachableBin.arm.planTargetName(), "CLASSIFICATION") == 0 &&
            unreachableBin.arm.planReport().ik.task == IkTask::PositionClosestOrientation &&
            positionCommands == 0 && gripperCommands == 0,
        2009);
}

extern "C" __attribute__((noinline, used)) void test_done() {
  for (;;)
    asm volatile("nop");
}
static void repeatedCartesianTargetsAndLeftDriveLoss() {
  RobotDHParams p = robot_get_dh_params();
  float finalXCommand[5]{};
  for (int axis = 0; axis < 3; ++axis) {
    float seed[5]{}, pose[6], homePose[6];
    robot_fkine(seed,p,pose); memcpy(homePose,pose,sizeof(pose));
    for (int step = 0; step < 32; ++step) {
      float delta[6]{}, actual[6]; delta[axis] = step < 16 ? 2.5f : -2.5f;
      IkReport report;
      bool ok = robot_ikine_increment(seed,pose,delta,250,.5f,.035f,p,report);
      check(ok,2101+axis);
      if (!ok) break;
      robot_fkine(seed,p,actual);
      check(hypotf(hypotf(actual[0]-pose[0],actual[1]-pose[1]),actual[2]-pose[2]) <= .5f,2110+axis);
      for (int other = 0; other < 3; ++other)
        if (other != axis) check(fabsf(actual[other]-homePose[other]) <= .5f,2120+axis);
      if (axis == 0 && step == 15) memcpy(finalXCommand,seed,sizeof(seed));
    }
  }
  // Independent differential relation: pitch=(R-L)/2, roll=(R+L)/2.
  // Simulate the observed left gear near HOME while the right follows its target.
  float right = finalXCommand[0]+finalXCommand[1], left = -.35f;
  float actual[5], pose[6]; memcpy(actual,finalXCommand,sizeof(actual));
  actual[0] = (right-left)/2; actual[1] = (right+left)/2;
  robot_fkine(actual,p,pose);
  check(pose[1] > 20 && pose[1] < 25 && pose[2] > 275 && pose[2] < 281,2130);
  const float desired[5] = {20,0,-13.06f,-32.40f,-89.91f};
  const float reported[5] = {10.16f,9.81f,-13.43f,-32.62f,-89.91f};
  float desiredPose[6], reportedPose[6];
  robot_fkine(desired,p,desiredPose); robot_fkine(reported,p,reportedPose);
  check(hypotf(hypotf(desiredPose[0]-332,desiredPose[1]),desiredPose[2]-51) <= .5f,2131);
  float error = hypotf(hypotf(desiredPose[0]-reportedPose[0],desiredPose[1]-reportedPose[1]),
                       desiredPose[2]-reportedPose[2]);
  check(error > 95 && error < 100,2132);
}
static void calibratedTrajectoryTestRange() {
  // Execute the real planner/controller, including the two classification
  // commands, at both range endpoints and intermediate pickup locations.
  const float xs[] = {200, 220, 240, 260, 280, 300, 320, 332, 340, 360, 380, 400, 420};
  for (int i = 0; i < 13; ++i)
    for (uint8_t bin = 1; bin <= 3; ++bin) {
    Fixture f;
    const AutoConfig base = f.config;
    f.config = TrajectoryTest::configuration(base, xs[i], f.params, bin);
    uint32_t code = 4000 + (3 * i + bin - 1) * 16;
    check(base.pickupZ == -34 && base.defaultDestination[0] == 0 &&
              f.config.approachZ == base.approachZ,
          code);
    check(f.pick(xs[i]) && f.until("DESCEND"), code + 1);
    if (!f.stage("DESCEND"))
      continue;
    float pickupPose[6];
    robot_fkine(commanded, f.params, pickupPose);
    check(hypotf(hypotf(pickupPose[0] - xs[i], pickupPose[1]),
                 pickupPose[2] - TrajectoryTest::beltContactZ(xs[i])) <= .5f,
          code + 2);
    float pickupQ5 = commanded[4];
    float pickupQ4 = commanded[3];
    check(fabsf(pickupPose[3] + TrajectoryTest::surfaceTiltDegrees()) <= .035f * 180 / PI &&
              fabsf(pickupPose[4]) <= .035f * 180 / PI &&
              fabsf(pickupPose[5] - 90) <= .035f * 180 / PI,
          code + 3);
    // Exercise movement/settling normally, then skip the stationary waiting
    // interval. Dedicated timing/pause tests cover every normal countdown tick.
    bool waiting = f.until("WAIT FOR PART");
    check(waiting, code + 9);
    if (!waiting)
      continue;
    uint32_t elapsed = f.now - f.startTime;
    f.tick(elapsed < 3500 ? 3500 - elapsed : 0);
    check(f.until("CLASSIFICATION WAYPOINT") && fabsf(commanded[4] - pickupQ5) < .005f &&
              fabsf(commanded[3] - pickupQ4) < .005f && commanded[0] == 0,
          code + 4);
    check(f.until("CLASSIFICATION POSE") && commanded[0] == 0,
          code + 5);
    check(f.finish() == PickOutcome::Sorted && positionCommands == 4 && gripperCommands == 2,
          code + 6);
    float dropPose[6];
    robot_fkine(commanded, f.params, dropPose);
    const float *target = f.config.defaultDestination;
    check(hypotf(hypotf(dropPose[0] - target[0], dropPose[1] - target[1]),
                 dropPose[2] - target[2]) <= .5f,
          code + 7);
    check(f.arm.planReport().ik.task == IkTask::FullPose &&
          fabsf(dropPose[3]-target[3])<2.5f && fabsf(dropPose[4]-target[4])<2.5f &&
          fabsf(dropPose[5]-target[5])<2.5f, code + 8);
    // The application now dispatches prepare() after Sorted and reports cycle
    // completion only on HomeReady. Exercise that return for every X/bin pair.
    check(f.arm.prepare(f.config, f.feedback, f.now) && f.stage("HOME") && f.arm.busy(),
          code + 10);
    check(positionCommands == 5 && gripperCommands == 3, code + 11);
    bool homeCommand = true;
    for (float q : commanded)
      homeCommand &= q == 0;
    check(homeCommand, code + 12);
    // Old classification feedback must not qualify as arrival at HOME.
    f.tick(100, false);
    check(f.arm.busy() && f.stage("HOME"), code + 13);
    check(f.finish() == PickOutcome::HomeReady && !f.arm.busy(), code + 14);
  }
}

static void jointLimitCommandPrecision() {
  params = robot_get_dh_params();
  limits_received = true;
  armFeedback.valid = true;
  armFeedback.timestampMs = adapterTime;
  float q[5] = {0, 0, -70, 0, 84};
  check(validAutoJoints(q), 2301);
  q[4] = 84.01f;
  check(!validAutoJoints(q), 2302);
  q[4] = -95;
  check(validAutoJoints(q), 2303);
  q[4] = -95.01f;
  check(!validAutoJoints(q), 2304);
}

static void heldQ5MustStillPreserveClassificationPosition() {
  Fixture f;
  f.config = TrajectoryTest::configuration(f.config, 332);
  // An off-axis future TCP makes q5 affect position. Constrain it inside IK
  // even with shoulder pitch unlocked, rather than modifying a solved pose.
  f.params.toolOffset[0] = 15;
  // This hypothetical tool cannot use the nominal full pickup orientation.
  // Relax pickup orientation so the test reaches the classification override.
  f.config.pickupTask = IkTask::PositionClosestOrientation;
  f.config.classificationPitchZero = f.config.classificationFixedShoulders = false;
  f.config.holdPickupQ5ForSorting=true;
  // Keep this intentionally incompatible fixture independent of the editable
  // bin configuration. Its heading is clear of the q5 joint limit.
  const float offAxisDestination[]={-76,391.5f,138.1f,0,-23.6f,20};
  memcpy(f.config.defaultDestination,offAxisDestination,sizeof(offAxisDestination));
  check(f.pick(),2380);
  const PickOutcome result=f.finish();const ArmPlanReport report=f.arm.planReport();
  // This off-axis target cannot meet the task with the actual pickup q5 held.
  // The solver must leave q5 fixed and reject before any arm/gripper command.
  check(result==PickOutcome::Rejected && report.ik.task==IkTask::PositionClosestOrientation &&
        report.ik.positionErrorMm>.5f && fabsf(report.candidate[4]-report.seed[4])<.005f,2381);
  check(positionCommands==0 && gripperCommands==0,2382);
}

static void binGeometryAndPartRouting() {
  AutoConfig config;
  RobotDHParams p = robot_get_dh_params();
  Classification::configure(config, p);
  // Independently check the editable reference/sweep against poseForBin's
  // Rodrigues implementation, rather than hard-coding an old bin layout.
  const float rx = Classification::referencePose[0] + 76;
  const float ry = Classification::referencePose[1];
  float radius = hypotf(rx, ry);
  const float referenceAngle = atan2f(ry, rx);
  for (int i = 0; i < 3; ++i) {
    const Destination *d = Classification::destinationForPart(config, partForBin(i + 1));
    check(d && d->binId == i + 1, 3500 + i);
    const float a = referenceAngle + Classification::bins[i].rollOffsetDegrees * PI / 180;
    const float expected[] = {-76 + radius*cosf(a), radius*sinf(a), Classification::referencePose[2]};
    for (int k = 0; k < 3; ++k)
      check(d && fabsf(d->pose[k] - expected[k]) < .001f, 3510 + i);
    check(d && fabsf(hypotf(d->pose[0] + 76, d->pose[1]) - radius) < .001f, 3520 + i);
  }
  check(!Classification::destinationForPart(config, 0) &&
            !Classification::destinationForPart(config, 4) &&
            !Classification::destinationForPart(config, 255), 3530);
  config.destinations[0].enabled = false;
  check(!Classification::destinationForPart(config, partForBin(1)) &&
            Classification::activeRouteCount(config) == 2, 3531);
  check(Classification::activeRoute(0, config)->partId == partForBin(2) &&
            Classification::activeRoute(1, config)->partId == partForBin(3) &&
            !Classification::activeRoute(2, config), 3532);
  const Classification::PartRoute grouped[] = {{10,1},{11,1},{12,1},{22,2},{99,0}};
  check(Classification::mappingValid(grouped, 5) &&
            Classification::binForPartId(10, grouped, 5) == 1 &&
            Classification::binForPartId(11, grouped, 5) == 1 &&
            Classification::binForPartId(99, grouped, 5) == 0 &&
            Classification::binForPartId(100, grouped, 5) == 0, 3533);
  const Classification::PartRoute duplicate[] = {{10,1},{10,2}};
  const Classification::PartRoute invalid[] = {{10,4}};
  check(!Classification::mappingValid(duplicate, 2) &&
            !Classification::mappingValid(invalid, 1), 3534);
  for (Destination &d : config.destinations)
    d.enabled = false;
  check(Classification::activeRouteCount(config) == 0 &&
            !Classification::activeRoute(0, config), 3535);
}

static void waypointHoldsMeasuredQ4() {
  Fixture f;
  check(f.pick() && f.until("CLOSE"), 3540);
  float measuredQ4 = f.feedback.q[3] + .1f;
  f.feedback.q[3] = measuredQ4;
  f.tick(f.config.closingDurationMs, false);
  check(f.stage("CLASSIFICATION WAYPOINT") && fabsf(commanded[3] - measuredQ4) < .001f, 3541);
  check(f.until("CLASSIFICATION POSE") && fabsf(commanded[3] - measuredQ4) > 1, 3542);
  check(f.finish() == PickOutcome::Sorted, 3543);
}

static void invalidFixedJointSeedIsRejected() {
  RobotDHParams p = robot_get_dh_params();
  float pose[6] = {-50,323,186.4f,0,0,0};
  float seed[5] = {0,0,0,0,-100};
  IkStepper ik;
  ik.begin(seed, pose, p, IkTask::PositionOnly, 150, .5f, .035f, 0x11);
  check(ik.state() == IkState::Failed && ik.report().failure == IkFailure::InvalidInput, 3550);
}

static void finalHomePauseAndFeedbackSupervision() {
  Fixture f;
  check(f.pick() && f.finish() == PickOutcome::Sorted, 3560);
  check(f.arm.prepare(f.config, f.feedback, f.now), 3561);
  f.arm.requestPause(f.now);
  f.tick(50, false);
  check(f.stage("HOME") && !f.arm.paused(), 3562);
  for (int i = 0; !f.arm.paused() && i < 20; ++i)
    f.tick();
  uint32_t id;
  check(f.arm.paused() && f.arm.busy() && f.arm.takeOutcome(id) == PickOutcome::None, 3563);
  f.tick(15000); // Continue fresh feedback while holding HOME, no timeout on pause.
  check(f.arm.paused() && f.arm.resume(f.now), 3564);
  check(f.finish() == PickOutcome::HomeReady, 3565);

  Fixture stale;
  check(stale.pick() && stale.finish() == PickOutcome::Sorted &&
            stale.arm.prepare(stale.config, stale.feedback, stale.now), 3566);
  stale.tick(stale.config.feedbackTimeoutMs + 1, false, false);
  check(stale.arm.takeOutcome(id) == PickOutcome::Fault, 3567);

  Fixture stopped;
  check(stopped.pick() && stopped.finish() == PickOutcome::Sorted &&
            stopped.arm.prepare(stopped.config, stopped.feedback, stopped.now), 3568);
  stopped.tick(stopped.config.motionTimeoutMs + 1, false);
  check(stopped.arm.takeOutcome(id) == PickOutcome::Fault &&
            strcmp(stopped.arm.fault(), "Stage timeout") == 0, 3569);
}

static void tunedHeightAndHomeWristAllowance() {
  Fixture f;
  f.config = TrajectoryTest::configuration(f.config, 332, f.params);
  f.config.settleMs = 150;
  f.config.openingDurationMs = 500;
  check(fabsf(TrajectoryTest::beltContactZ(PickMotion::heightReferenceXmm) -
              (PickMotion::fittedContactZmm + PickMotion::pickupHeightAdjustmentMm)) < .0001f, 6000);
  check(f.config.jointToleranceDeg == 1 && f.config.positionToleranceMm == 5 &&
            f.config.useHomeWaypointArrival, 6001);
  check(f.arm.prepare(f.config, f.feedback, f.now), 6002);
  const float recordedHome[5] = {.06f, .32f, -.23f, -1.42f, 0};
  memcpy(f.feedback.q, recordedHome, sizeof(recordedHome));
  f.tick(50, false);
  ArmProgress p = f.arm.progress(f.feedback, f.now);
  check(p.withinEntry && p.withinTarget && p.positionErrorMm < 5 &&
            p.jointToleranceDeg[3] == 1.5f, 6003);
  f.tick(150, false);
  p = f.arm.progress(f.feedback, f.now);
  check(f.arm.busy() && p.stableMs == 150, 6004);
  f.tick(300, false);
  uint32_t id;
  check(f.arm.takeOutcome(id) == PickOutcome::HomeReady, 6005);

  Fixture strict;
  check(strict.arm.prepare(strict.config, strict.feedback, strict.now), 6006);
  memcpy(strict.feedback.q, recordedHome, sizeof(recordedHome));
  strict.tick(50, false);
  p = strict.arm.progress(strict.feedback, strict.now);
  check(!p.withinTarget && p.positionErrorMm < 5 && p.jointToleranceDeg[3] == 1,
        6007); // Production retains its original joint requirement.
}

// Find a small pitch deviation from the command with a known TCP distance. This gives
// controlled 4.8/5.4/6.2 mm samples without assuming a linear FK scale.
static void targetTcpError(Fixture &f, float distance) {
  float pose[6], targetPose[6], q[5];
  memcpy(q, commanded, sizeof(q));
  robot_fkine(q, f.params, targetPose);
  float low = 0, high = 1;
  for (int i = 0; i < 18; ++i) {
    q[0] = commanded[0] + (low + high) / 2;
    robot_fkine(q, f.params, pose);
    float error = hypotf(hypotf(pose[0] - targetPose[0], pose[1] - targetPose[1]),
                         pose[2] - targetPose[2]);
    if (error < distance)
      low = q[0] - commanded[0];
    else
      high = q[0] - commanded[0];
  }
  memcpy(f.feedback.q, q, sizeof(q));
}

static void arrivalHysteresisEntryResetAndFreshSamples() {
  Fixture f;
  f.config = TrajectoryTest::configuration(f.config, 332, f.params);
  f.config.settleMs = 150;
  f.config.openingDurationMs = 1000;
  check(f.arm.prepare(f.config, f.feedback, f.now), 6020);
  targetTcpError(f, 5.4f);
  f.tick(50, false);
  ArmProgress p = f.arm.progress(f.feedback, f.now);
  check(!p.withinEntry && !p.withinTarget && p.stableMs == 0, 6021);
  targetTcpError(f, 4.8f);
  f.tick(50, false);
  check(f.arm.progress(f.feedback, f.now).withinEntry, 6022);
  targetTcpError(f, 5.4f);
  f.tick(50, false);
  p = f.arm.progress(f.feedback, f.now);
  check(!p.withinEntry && p.withinTarget && p.stableMs == 50 &&
            p.positionEntryToleranceMm == 5 && p.positionHoldToleranceMm == 6, 6023);
  targetTcpError(f, 6.2f);
  f.tick(50, false);
  check(!f.arm.progress(f.feedback, f.now).withinTarget &&
            f.arm.progress(f.feedback, f.now).stableMs == 0, 6024);
  targetTcpError(f, 5.4f);
  f.tick(50, false);
  check(!f.arm.progress(f.feedback, f.now).withinTarget, 6025);
  targetTcpError(f, 4.8f);
  f.tick(50, false);
  targetTcpError(f, 5.4f);
  f.tick(50, false);
  f.tick(100, false, false); // Same feedback sequence/timestamp cannot add settling time.
  check(f.arm.progress(f.feedback, f.now).stableMs == 50 && f.arm.busy(), 6026);
  targetTcpError(f, 0);
  f.feedback.q[3] = 1.51f;
  f.tick(50, false);
  p = f.arm.progress(f.feedback, f.now);
  check(p.positionErrorMm < 5 && !p.withinTarget && p.stableMs == 0, 6027);
}

static void waypointLogReplayAndStrictFinalDropTcp() {
  Fixture f;
  f.config = TrajectoryTest::configuration(f.config, 332, f.params);
  f.config.settleMs = 150;
  check(f.pick() && f.until("CLASSIFICATION WAYPOINT"), 6040);
  // Reproduce the supplied logged geometry separately: its random X need not
  // match this test's X=332, so the commanded waypoint joints differ.
  const float recordedTarget[5] = {0, 0, 1.81f, -44.38f, -90};
  float actual[5], targetPose[6], actualPose[6], distances[2];
  robot_fkine(recordedTarget, f.params, targetPose);
  const float error[5] = {.36f, .03f, -.24f, 0, 0};
  for (int i = 0; i < 5; ++i)
    actual[i] = recordedTarget[i] + error[i];
  for (int sample = 0; sample < 2; ++sample) {
    actual[3] = recordedTarget[3] + (sample ? .24f : 0);
    robot_fkine(actual, f.params, actualPose);
    distances[sample] = hypotf(hypotf(actualPose[0] - targetPose[0],
                                     actualPose[1] - targetPose[1]),
                               actualPose[2] - targetPose[2]);
  }
  check(distances[0] < 5 && distances[1] > 5 && distances[1] < 6, 6046);
  targetTcpError(f, 4.8f);
  f.tick(50, false);
  ArmProgress p = f.arm.progress(f.feedback, f.now);
  check(p.withinEntry && p.positionErrorMm > 4 && p.positionErrorMm < 5, 6041);
  targetTcpError(f, 5.4f);
  f.tick(50, false);
  p = f.arm.progress(f.feedback, f.now);
  check(!p.withinEntry && p.withinTarget && p.positionErrorMm > 5 &&
            p.positionErrorMm < 6 && p.stableMs == 50, 6042);
  f.tick(50, false);
  check(f.stage("CLASSIFICATION WAYPOINT") &&
            f.arm.progress(f.feedback, f.now).stableMs == 100, 6043);
  f.tick(50, false);
  p = f.arm.progress(f.feedback, f.now);
  check(f.stage("CLASSIFICATION POSE") && p.positionEntryToleranceMm == 5 &&
            p.positionHoldToleranceMm == 5 && p.jointToleranceDeg[3] == 1.5f, 6044);
  check(f.finish() == PickOutcome::Sorted, 6045);
}

static void pausedHysteresisStillDetectsDriftAndStaleFeedback() {
  Fixture f;
  f.config = TrajectoryTest::configuration(f.config, 332, f.params);
  f.config.settleMs = 150;
  check(f.arm.prepare(f.config, f.feedback, f.now), 6060);
  f.arm.requestPause(f.now);
  targetTcpError(f, 4.8f);
  f.tick(50, false);
  targetTcpError(f, 5.4f);
  for (int i = 0; i < 3; ++i)
    f.tick(50, false);
  check(f.arm.paused() && f.arm.busy(), 6061);
  f.tick(50, false);
  check(f.arm.paused(), 6062);
  targetTcpError(f, 6.2f);
  f.tick(50, false);
  uint32_t id;
  check(f.arm.takeOutcome(id) == PickOutcome::Fault &&
            strcmp(f.arm.fault(), "Paused pose lost") == 0, 6063);
  ArmProgress p = f.arm.progress(f.feedback, f.now);
  check(p.positionHoldToleranceMm == 6 && p.jointToleranceDeg[3] == 2.0f, 6064);
  Fixture stale;
  stale.config = TrajectoryTest::configuration(stale.config, 332, stale.params);
  check(stale.arm.prepare(stale.config, stale.feedback, stale.now), 6065);
  stale.tick(stale.config.feedbackTimeoutMs + 1, false, false);
  check(stale.arm.takeOutcome(id) == PickOutcome::Fault &&
            strcmp(stale.arm.fault(), "Nucleo feedback stale") == 0, 6066);
}

static void invalidArrivalSettingsNeverMove() {
  for (int i = 0; i < 11; ++i) {
    Fixture f;
    f.config = TrajectoryTest::configuration(f.config, 332, f.params);
    if (i == 0) f.config.homeWaypointArrival.jointToleranceDeg[3] = NAN;
    if (i == 1) f.config.homeWaypointArrival.positionHysteresisMm = -1;
    if (i == 2) f.config.homeWaypointArrival.positionToleranceMm = INFINITY;
    if (i == 3) f.config.homeWaypointArrival.jointToleranceDeg[0] = 0;
    if (i == 4) f.config.positionToleranceMm = NAN;
    if (i == 5) f.config.jointToleranceOverrideDeg[3] = NAN;
    if (i == 6) f.config.jointToleranceOverrideDeg[3] = INFINITY;
    if (i == 7) f.config.jointToleranceOverrideDeg[3] = -1;
    if (i == 8) f.config.jointHoldHysteresisDeg[3] = -1;
    if (i == 9) f.config.jointHoldHysteresisDeg[3] = NAN;
    if (i == 10) f.config.jointHoldHysteresisDeg[3] = INFINITY;
    check(!validArrivalConfiguration(f.config) &&
              !f.arm.prepare(f.config, f.feedback, f.now), 6080 + i);
    uint32_t id;
    check(f.arm.takeOutcome(id) == PickOutcome::Fault && positionCommands == 0 &&
              gripperCommands == 0, 6090 + i);
    check(!f.pick() && f.arm.takeOutcome(id) == PickOutcome::Fault &&
              positionCommands == 0 && gripperCommands == 0, 6100 + i);
  }
}

static void tickWithWristError(Fixture &f, float error, uint32_t elapsed = 50) {
  memcpy(f.feedback.q, commanded, sizeof(commanded));
  f.feedback.q[3] += error;
  f.tick(elapsed, false);
}
// Isolate the joint hold boundary from the independent 5mm TCP boundary.
// Choose a small elbow residual within its unchanged 1deg tolerance.
static void tickWithCompensatedWristError(Fixture &f,float wristError) {
  float expected[6],actual[6];robot_fkine(commanded,f.params,expected);
  memcpy(f.feedback.q,commanded,sizeof(commanded));f.feedback.q[3]+=wristError;
  float best=INFINITY,elbowError=0;
  for(int k=-90;k<=90;++k) {
    f.feedback.q[2]=commanded[2]+k*.01f;
    robot_fkine(f.feedback.q,f.params,actual);
    const float error=hypotf(hypotf(actual[0]-expected[0],actual[1]-expected[1]),actual[2]-expected[2]);
    if(error<best) {best=error;elbowError=k*.01f;}
  }
  f.feedback.q[2]=commanded[2]+elbowError;f.tick(50,false);
}
static void wristHoldRequiresConfirmedEntryAndResetsForEveryMove() {
  Fixture f;
  f.config=TrajectoryTest::configuration(f.config,289,f.params);
  f.config.pickUpMarginMs=2500; f.config.descentLeadMs=760; f.config.closingLeadMs=420;
  f.config.settleMs=150; f.config.minimumMoveMs=200;
  check(f.pick(289) && f.until("APPROACH"),9600);
  for(unsigned i=0;i<5;++i)tickWithWristError(f,-1.51f);
  ArmProgress p=f.arm.progress(f.feedback,f.now);
  check(f.stage("APPROACH") && !p.withinEntry && !p.withinTarget && !p.jointHoldActive &&
        p.jointToleranceDeg[3]==1.5f && p.jointHoldToleranceDeg[3]==2.0f,9601);
  tickWithWristError(f,-1.0f); // Enter, but not yet confirmed/settled.
  tickWithWristError(f,-1.51f);
  p=f.arm.progress(f.feedback,f.now);
  check(!p.jointHoldActive && p.stableMs==0 && f.stage("APPROACH"),9602);
  for(unsigned i=0;i<4;++i)tickWithWristError(f,-1.0f);
  check(f.stage("SAFETY HEIGHT WAIT") && f.arm.arrivalDelayMs(f.now)>760,9603);
  const float holdingErrors[]={-1.51f,-1.75f,-1.99f};
  for(float e:holdingErrors) {
    tickWithCompensatedWristError(f,e);p=f.arm.progress(f.feedback,f.now);
    check(f.stage("SAFETY HEIGHT WAIT") && !p.withinEntry && p.withinTarget && p.jointHoldActive &&
          p.jointToleranceDeg[3]==2.0f && p.positionHoldToleranceMm==5,9604);
  }
  tickWithCompensatedWristError(f,-2.05f);
  check(f.stage("MISSED-PICK RECOVERY") && !strcmp(f.arm.missReason(),"SAFETY_HEIGHT_LOST") &&
        !sawClose,9605);
  ArmDiagnosticReport d;bool found=false;
  while(f.arm.takeDiagnostic(d)) if(d.event==ArmDiagnosticEvent::Missed) {
    found=true;
    check(d.progress.jointHoldActive && d.progress.jointToleranceDeg[3]==2.0f &&
          d.progress.jointEntryToleranceDeg[3]==1.5f && !d.progress.withinTarget &&
          d.progress.positionErrorMm<5 && fabsf(d.progress.error[2])<1,9606);
    Serial.clear();printArmDiagnostic(Serial,d);
    check(strstr(Serial.text,"q4entry/hold=1.50/2.00deg holdActive=1")!=nullptr,9607);
  }
  check(found,9608);
  p=f.arm.progress(f.feedback,f.now);
  check(!p.jointHoldActive && p.jointToleranceDeg[3]==1.5f,9609); // Recovery is a new target.
  check(f.finish()==PickOutcome::Missed,9610);
}
static void wristHoldingStillRequiresTcpOtherJointsAndFreshFeedback() {
  for(unsigned scenario=0;scenario<4;++scenario) {
    Fixture f;
    f.config=TrajectoryTest::configuration(f.config,289,f.params);
    f.config.settleMs=150;f.config.minimumMoveMs=200;
    check(f.pick(289) && f.until("SAFETY HEIGHT WAIT"),9620+scenario);
    if(scenario==0) {
      targetTcpError(f,5.4f);f.tick(50,false);
      check(f.stage("MISSED-PICK RECOVERY") && !strcmp(f.arm.missReason(),"SAFETY_HEIGHT_LOST"),9630);
    } else if(scenario==1) {
      memcpy(f.feedback.q,commanded,sizeof(commanded));f.feedback.q[2]+=1.01f;f.tick(50,false);
      check(f.stage("MISSED-PICK RECOVERY"),9631);
    } else if(scenario==2) {
      f.arm.requestPause(f.now);f.tick();
      tickWithWristError(f,-1.51f);
      check(f.arm.paused() && f.arm.busy(),9632);
      tickWithWristError(f,-2.05f);uint32_t id;
      check(f.arm.takeOutcome(id)==PickOutcome::Fault && !strcmp(f.arm.fault(),"Paused pose lost"),9633);
    } else {
      tickWithWristError(f,-1.51f);
      f.tick(f.config.feedbackTimeoutMs+1,false,false);uint32_t id;
      check(f.arm.takeOutcome(id)==PickOutcome::Fault && !strcmp(f.arm.fault(),"Nucleo feedback stale"),9634);
    }
  }
  Fixture f;
  f.config=TrajectoryTest::configuration(f.config,289,f.params);
  f.config.openingDurationMs=500;
  check(f.pick(289) && f.until("WAIT FOR PART"),9640);
  tickWithWristError(f,-1.51f);
  check(f.stage("WAIT FOR PART") && f.arm.progress(f.feedback,f.now).jointHoldActive,9641);
  check(f.until("OPEN"),9642);
  tickWithWristError(f,-1.51f);
  check(f.stage("OPEN") && f.arm.progress(f.feedback,f.now).withinTarget,9643);
  tickWithWristError(f,-2.05f);uint32_t id;
  check(f.arm.takeOutcome(id)==PickOutcome::Fault &&
        !strcmp(f.arm.fault(),"Classification pose lost while opening"),9644);
}
static void approachWristDitherNoLongerBlocksTest() {
  Fixture f;
  f.config = TrajectoryTest::configuration(f.config, 289, f.params);
  f.config.settleMs = 150;
  f.config.minimumMoveMs = 200;
  check(f.pick(289) && f.until("APPROACH"), 7200);
  float approach[5];
  memcpy(approach, commanded, sizeof(approach));
  const float error[5] = {.21f, .09f, -.20f, -1.02f, 0};
  for (int i = 0; i < 5; ++i)
    f.feedback.q[i] = approach[i] + error[i];
  f.tick(50, false);
  ArmProgress p = f.arm.progress(f.feedback, f.now);
  check(p.feedbackValid && p.withinTarget && p.positionErrorMm < 5 &&
            p.jointToleranceDeg[3] == 1.5f && p.positionHoldToleranceMm == 5, 7201);
  for (int sample = 1; sample <= 3; ++sample) {
    f.feedback.q[3] = approach[3] + (sample % 2 ? -.78f : -1.02f);
    f.tick(50, false);
  }
  check(f.stage("SAFETY HEIGHT WAIT") && uint32_t(f.now - f.startTime) < 3400, 7202);
  // Continue with the same observed wrist residual through pickup, wait,
  // closing, the waypoint, final drop and opening. TCP checks stay active.
  bool sawWait = false, sawClosing = false, sawMid = false, sawDrop = false, sawOpen = false;
  for (int i = 0; f.arm.busy() && i < 300; ++i) {
    sawWait |= f.stage("WAIT FOR PART");
    sawClosing |= f.stage("CLOSE");
    sawMid |= f.stage("CLASSIFICATION WAYPOINT");
    sawDrop |= f.stage("CLASSIFICATION POSE");
    sawOpen |= f.stage("OPEN");
    tickWithWristError(f, i % 2 ? -.78f : -1.02f);
  }
  uint32_t id;
  check(f.arm.takeOutcome(id) == PickOutcome::Sorted && sawClosing &&
            sawMid && sawDrop && sawOpen, 7203);
  check(sawClose && uint32_t(closeTime - f.startTime) >= 3500 &&
            uint32_t(closeTime - f.startTime) <= 3550, 7204);
}
static void strictWristLimitReproducesReportedTimeout() {
  Fixture f;
  f.config = TrajectoryTest::configuration(f.config, 289, f.params);
  f.config.jointToleranceOverrideDeg[3] = 0; // Reproduce the previous 1-degree policy.
  f.config.settleMs = 150;
  check(f.pick(289) && f.until("APPROACH"), 7220);
  tickWithWristError(f, -1.02f);
  ArmProgress p = f.arm.progress(f.feedback, f.now);
  check(!p.withinTarget && p.positionErrorMm < 5 && p.jointToleranceDeg[3] == 1, 7221);
  bool recovered = false;
  for (int i = 0; f.arm.busy() && i < 300; ++i) {
    recovered |= f.stage("MISSED-PICK RECOVERY");
    tickWithWristError(f, i % 2 ? -.78f : -1.02f);
  }
  uint32_t id;
  check(recovered && !sawClose && f.arm.takeOutcome(id) == PickOutcome::Fault &&
            strcmp(f.arm.faultStage(), "MISSED-PICK RECOVERY") == 0 &&
            strcmp(f.arm.fault(), "Stage timeout") == 0, 7222);
}
static void recoveryWristAllowanceStillChecksTcpAndJointError() {
  Fixture f;
  f.config = TrajectoryTest::configuration(f.config, 289, f.params);
  f.config.settleMs = 150;
  check(f.pick(289) && f.until("APPROACH"), 7240);
  f.tick(3600, false); // Genuinely late approach still misses the pick.
  check(f.stage("MISSED-PICK RECOVERY") && !sawClose, 7241);
  for (int i = 0; f.arm.busy() && i < 20; ++i)
    tickWithWristError(f, i % 2 ? -.78f : -1.02f);
  uint32_t id;
  check(f.arm.takeOutcome(id) == PickOutcome::Missed && !sawClose, 7242);

  Fixture outside;
  outside.config = TrajectoryTest::configuration(outside.config, 289, outside.params);
  check(outside.pick(289) && outside.until("APPROACH"), 7243);
  tickWithWristError(outside, -1.51f);
  ArmProgress p = outside.arm.progress(outside.feedback, outside.now);
  check(p.positionErrorMm < 5 && !p.withinTarget, 7244);
  memcpy(outside.feedback.q, commanded, sizeof(commanded));
  outside.feedback.q[0] += .9f;
  outside.tick(50, false);
  p = outside.arm.progress(outside.feedback, outside.now);
  check(p.positionErrorMm > 5 && !p.withinTarget &&
            p.positionHoldToleranceMm == 5 && p.jointToleranceDeg[0] == 1, 7245);
}

static calibH identityCalibration() {
  calibH c;
  c.valid = true;
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col)
      c.H(row, col) = row == col ? 1 : 0;
  return c;
}
static objectVision calibrationDetection(uint16_t x, uint16_t y) {
  objectVision d{};
  d.xCenter = x;
  d.yCenter = y;
  d.width = d.height = 20;
  d.ID = 1;
  return d;
}
static void cameraRotationKeepsWorldAxes() {
  const float worldX[4] = {TAG0_WORLD_X, TAG1_WORLD_X, TAG2_WORLD_X, TAG3_WORLD_X};
  const float worldY[4] = {TAG0_WORLD_Y, TAG1_WORLD_Y, TAG2_WORLD_Y, TAG3_WORLD_Y};
  for (int rotated = 0; rotated < 2; ++rotated) {
    float u[4], v[4];
    for (int i = 0; i < 4; ++i) {
      u[i] = rotated ? .5f * worldY[i] - 100 : .5f * worldX[i] + 20;
      v[i] = rotated ? .5f * (650 - worldX[i]) + 30 : .5f * worldY[i] - 100;
    }
    calibH c = AutoHomography(u, v);
    check(c.valid, 7000 + rotated);
    BeltCalibrator b(0, fixtureVelocityPixels); // Isolate homography rotation.
    for (int i = 0; i <= 10; ++i) {
      float y = 600 - 10 * i;
      objectVision d = rotated ? calibrationDetection(uint16_t(.5f * y - 100), 189)
                               : calibrationDetection(186, uint16_t(.5f * y - 100));
      check(b.add(d, 1000 + i * 100, c), 7010 + rotated);
    }
    float velocity = 0;
    BeltCalibrationReport r = b.report();
    check(b.finish(velocity) && r.issue == BeltCalibrationIssue::None &&
              fabsf(velocity + 100) < .05f && fabsf(r.deltaXmm) < .02f &&
              fabsf(r.worldX - 332) < .02f && fabsf(r.worldY - 500) < .02f, 7020 + rotated);
  }
}
static void velocityCalibrationRejectionReasons() {
  calibH c = identityCalibration();
  BeltCalibrator b(0, fixtureVelocityPixels); // Synthetic pixels = world mm.
  float velocity = 123;
  check(!b.finish(velocity) && velocity == 123 && !b.seen() &&
            b.report().issue == BeltCalibrationIssue::TooFewSamples, 7040);
  for (int i = 0; i < 8; ++i)
    b.add(calibrationDetection(332, 600 - 8 * i), 1000 + i * 80, c);
  BeltCalibrationReport r = b.report();
  check(!b.finish(velocity) && r.issue == BeltCalibrationIssue::ShortSpan &&
            r.samples == 8 && r.spanMs == 560 && r.travelMm == 56, 7041);
  b.reset();
  check(!b.failed() && !b.seen() && b.samples() == 0, 7042);
  for (int i = 0; i <= 10; ++i)
    b.add(calibrationDetection(332, 600 - i), 1000 + i * 100, c);
  check(!b.finish(velocity) && b.report().issue == BeltCalibrationIssue::ShortTravel, 7043);
  b.reset();
  b.add(calibrationDetection(332, 600), 1000, c);
  check(!b.add(calibrationDetection(353, 590), 1100, c) && b.failed() &&
            b.report().issue == BeltCalibrationIssue::LateralJump &&
            b.report().worldX == 353 && b.report().samples == 1, 7044);
  check(!b.add(calibrationDetection(332, 580), 1200, c) &&
            b.report().issue == BeltCalibrationIssue::LateralJump, 7045);
  b.reset();
  b.add(calibrationDetection(332, 600), 1000, c);
  check(!b.add(calibrationDetection(332, 611), 1100, c) &&
            b.report().issue == BeltCalibrationIssue::ReverseStep, 7046);
  b.reset();
  b.add(calibrationDetection(332, 600), 1000, c);
  check(!b.add(calibrationDetection(332, 500), 2501, c) &&
            b.report().issue == BeltCalibrationIssue::SampleGap, 7047);
  b.reset();
  c.valid = false;
  check(!b.add(calibrationDetection(332, 600), 1000, c) &&
            b.report().issue == BeltCalibrationIssue::NonfiniteProjection, 7048);
  c.valid = true;
  b.reset();
  for (int i = 0; i <= 10; ++i)
    b.add(calibrationDetection(332, 600 - i * 10 + (i % 2 ? -9 : 9)), 1000 + i * 100, c);
  check(!b.failed() && !b.finish(velocity) &&
            b.report().issue == BeltCalibrationIssue::NoisyFit &&
            b.report().rmsMm > BeltCalibrationLimits::maximumRmsMm, 7049);
  b.reset();
  b.add(calibrationDetection(332, 600), 1000, c);
  for (int i = 1; i <= 30; ++i)
    b.add(calibrationDetection(332, 500 + 2 * (i - 1)), 1000 + i * 100, c);
  check(!b.failed() && !b.finish(velocity) &&
            b.report().issue == BeltCalibrationIssue::WrongDirection &&
            b.report().velocityMmS > 0 && b.report().travelMm > 20, 7050);
}
static void velocitySampleThrottlingAndClockWrap() {
  calibH c = identityCalibration();
  BeltCalibrator b(CameraTracking::partXOffsetMm, fixtureVelocityPixels);
  for (int i = 0; i <= 50; ++i)
    b.add(calibrationDetection(332, 600 - i * 2), 1000 + i * 20, c);
  float velocity = 0;
  BeltCalibrationReport r = b.report();
  check(b.finish(velocity) && r.samples == 13 && r.spanMs == 960 &&
            fabsf(velocity + 100) < .01f, 7070);
  b.reset();
  for (int i = 0; i <= 10; ++i)
    b.add(calibrationDetection(332, 600 - i * 10), uint32_t(0xffffff00U + i * 100), c);
  check(b.finish(velocity) && b.report().spanMs == 1000 &&
            fabsf(velocity + 100) < .01f, 7071);
  b.reset();
  const uint32_t times[] = {0, 100, 200, 300, 400, 500, 600, 800};
  for (uint32_t t : times)
    b.add(calibrationDetection(332, 600 - t / 10), 1000 + t, c);
  check(b.finish(velocity) && b.report().samples == 8 && b.report().spanMs == 800 &&
            fabsf(velocity + 100) < .01f, 7072);
}

static void calibrationRmsTrialAcceptsModerateScatterOnly() {
  calibH c = identityCalibration();
  BeltCalibrator b(CameraTracking::partXOffsetMm, fixtureVelocityPixels);
  for (int i = 0; i <= 10; ++i)
    check(b.add(calibrationDetection(332, 600 - i * 10 + (i % 2 ? -7 : 7)),
                1000 + i * 100, c), 7090);
  BeltCalibrationReport r = b.report();
  float velocity = 0;
  check(r.rmsMm > 5 && r.rmsMm < 8 && r.issue == BeltCalibrationIssue::None &&
            b.finish(velocity) && fabsf(velocity + 100) < .01f, 7091);
  // The calibration allowance does not widen the arm's pickup TCP check.
  Fixture f;
  f.config = TrajectoryTest::configuration(f.config, 289, f.params);
  check(f.pick(289) && f.until("APPROACH"), 7092);
  ArmProgress p = f.arm.progress(f.feedback, f.now);
  check(p.positionEntryToleranceMm == 5 && p.positionHoldToleranceMm == 5, 7093);
}

static AutoConfig automaticConfiguration() {
  AutoConfig c;
  c.closingLeadMs = 500; // Admission-boundary tests use a fixed timing reference.
  c.descentLeadMs = 600;
  Classification::configure(c, robot_get_dh_params());
  PickMotion::apply(c, PickMotion::heightReferenceXmm);
  c.pickUpMarginMs = 1000;
  return c;
}
static void automaticAdmissionUsesRealSnapshotsAndSharedGeometry() {
  AutoConfig base = automaticConfiguration();
  Part p;
  p.trackId = 42;
  p.inUse = true;
  p.hits = 3;
  p.y = 661.57f;
  p.stateMs = 1000;
  p.classVotes = CameraTracking::ClassificationConfidence::minimumSamples;
  p.classSinceMs = 1000 - CameraTracking::ClassificationConfidence::minimumStableMs;
  p.classLastMs = 1000;
  AutoPick::Request request;
  const float xs[] = {200,220,240,260,280,300,320,332,340,360,380,400,420};
  for (float x : xs)
    for (uint8_t bin = 1; bin <= 3; ++bin) {
      const uint8_t id = partForBin(bin);
      p.x = x;
      p.classId = p.pendingClass = id;
      check(AutoPick::evaluate(p, -125.71f, base, true, 1000, request) ==
                AutoPick::Admission::Eligible, 8000);
      AutoConfig test = TrajectoryTest::configuration(base, x, robot_get_dh_params(), bin);
      check(request.part.trackId == 42 && request.part.y == p.y && request.binId == bin &&
                request.config.pickupZ == test.pickupZ &&
                request.config.pickupRoll == test.pickupRoll &&
                request.config.pickupPitch == test.pickupPitch &&
                request.config.pickupYaw == test.pickupYaw &&
                request.config.pickupTask == IkTask::FullPose &&
                request.config.jointToleranceOverrideDeg[3] == test.jointToleranceOverrideDeg[3] &&
                request.config.homeWaypointArrival.positionHysteresisMm ==
                    test.homeWaypointArrival.positionHysteresisMm &&
                request.config.pickUpBudgetMs() == 1600 && test.pickUpBudgetMs() == 1600, 8001);
      check(memcmp(request.destination, test.defaultDestination, sizeof(request.destination)) == 0 &&
                request.closingDelayMs > 4700 && request.closingDelayMs < 4800, 8002);
    }
  p.x = PickMotion::minimumXmm - .1f;
  check(AutoPick::evaluate(p, -100, base, true, 1000, request) ==
            AutoPick::Admission::OutsidePickupRange, 8003);
  p.x = PickMotion::maximumXmm + .1f;
  check(AutoPick::evaluate(p, -100, base, true, 1000, request) ==
            AutoPick::Admission::OutsidePickupRange, 8004);
  p.x = 332;
  p.classId = p.pendingClass = 99;
  check(AutoPick::evaluate(p, -100, base, true, 1000, request) ==
            AutoPick::Admission::IgnoredId, 8005);
  check(AutoPick::evaluate(p, -100, base, false, 1000, request) ==
            AutoPick::Admission::Eligible && request.binId == 3 &&
            memcmp(request.destination, base.defaultDestination, sizeof(request.destination)) == 0, 8006);
  p.classId = p.pendingClass = partForBin(2);
  base.destinations[1].enabled = false;
  check(AutoPick::evaluate(p, -100, base, true, 1000, request) ==
            AutoPick::Admission::IgnoredId, 8007);
  base.destinations[1].enabled = true;
  p.y = 160;
  check(AutoPick::evaluate(p, -100, base, true, 1000, request) ==
            AutoPick::Admission::Eligible && request.arrivalDelayMs == 1600, 8008);
  check(AutoPick::evaluate(p, -100, base, true, 1001, request) ==
            AutoPick::Admission::TooLate, 8009);
  p.y = 600;
  check(AutoPick::evaluate(p, -100, base, true, 1250, request) ==
            AutoPick::Admission::Eligible && request.closingDelayMs == 5250, 8010);
  check(AutoPick::evaluate(p, 100, base, true, 1000, request) == AutoPick::Admission::Invalid &&
            AutoPick::evaluate(p, NAN, base, true, 1000, request) == AutoPick::Admission::Invalid, 8011);
  p.stateMs = 0xffffff00U;
  check(AutoPick::evaluate(p, -100, base, true, uint32_t(p.stateMs + 250), request) ==
            AutoPick::Admission::Eligible && request.closingDelayMs == 5250, 8012);
}

static float autoHarnessWristError;
static void setupAutoHarness(float velocity = -125.71f, bool visionOnly = false,
                             bool respectId = true) {
  params = robot_get_dh_params();
  autoConfig = automaticConfiguration();
  activeTuning=RuntimeTuning::defaults();
  activeTuning.profile(0)=PickupTuning(autoConfig);activeTuning.copyLegacyToBoth();
  tuningDraft=activeTuning;
  autoTuningLaunchPending=pickupSaveRequested=pickupEditorLocked=pickupEditorNeedsNeutral=false;
  tuningMenuRequested=false;
  arm = ArmTrajectory{};
  arm.bind(params, {sendJoints, sendGripper});
  mainTracker = BeltTracker(CameraTracking::partXOffsetMm, fixtureVelocityPixels);
  mainTracker.setVelocity(velocity);
  camera_calib = identityCalibration();
  armFeedback = ArmFeedback{};
  adapterTime = ioTime = 10000;
  armFeedback.timestampMs = adapterTime;
  armFeedback.sequence = 1;
  armFeedback.valid = limits_received = true;
  Serial1 = MockUart{};
  memset(commanded, 0, sizeof(commanded));
  positionCommands = gripperCommands = closeTime = 0;
  sawClose = false;
  currentState = AUTO_MODE;
  lastState = MODE_SELECTOR;
  autoRun = AutoRun::Stopped;
  menu_option0 = respectId ? 1 : 0;
  menu_option2 = visionOnly ? 1 : 0;
  sortedParts = rejectedParts = ignoredParts = outsidePickupParts = 0;
  auto_parts_missed = 0;
  lastArmProgressMs = totalObjects = visionTimestampMs = 0;
  homographyDone = velocityCalibrated = udpControllerStable = true;
  cameraOffline = communicationError = false;
  streamStarted = stopRequested = exitRequested = newLatest = false;
  entered_ES = leave_ES = holdingResetCombo = messageAck = false;
  controllerTimedOutThisLoop = controllerEmergencyReceived = false;
  currentControlCMD.buttons = 0;
  currentControlCMD.jsLx=currentControlCMD.jsLy=0;
  pendingESPmsg = criticalMessage{};
  fakeAckType = lastCriticalType = criticalCommands = 0;
  autoHarnessWristError = 0;
}
static void autoTick(uint32_t elapsed = 20, uint16_t pressed = 0,
                     bool follow = true, bool fresh = true) {
  adapterTime += elapsed;
  ioTime = adapterTime;
  if (follow) {
    memcpy(armFeedback.q, commanded, sizeof(commanded));
    armFeedback.q[3] += autoHarnessWristError;
  }
  if (fresh) {
    armFeedback.timestampMs = adapterTime;
    ++armFeedback.sequence;
  }
  serviceGlobalFaults();
  if (currentState == AUTO_MODE || currentState == AUTO_TUNING)
    serviceAutomaticMode(adapterTime, pressed);
}
static bool autoStage(const char *name) {
  return strcmp(arm.phaseName(), name) == 0;
}
static bool autoUntil(const char *name) {
  for (int i = 0; (currentState == AUTO_MODE || currentState == AUTO_TUNING) && i < 1000; ++i) {
    if (autoStage(name))
      return true;
    autoTick();
  }
  return false;
}
static bool startAutoHarness() {
  autoTick(0);
  check(autoRun == AutoRun::SyncStop && lastCriticalType == CMD_END_STREAM &&
            positionCommands == 0 && gripperCommands == 0, 8100);
  fakeAckType = CMD_END_STREAM;
  autoTick();
  check(autoRun == AutoRun::Stopped && !streamStarted, 8101);
  autoTick(20, BTN_A);
  check(autoRun == AutoRun::Starting && lastCriticalType == CMD_START_STREAM &&
            positionCommands == 0, 8102);
  fakeAckType = CMD_START_STREAM;
  autoTick();
  if (!autoMovesArm())
    return autoRun == AutoRun::Running && positionCommands == 0 && gripperCommands == 0;
  check(autoRun == AutoRun::Preparing && autoStage("HOME"), 8103);
  for (int i = 0; autoRun == AutoRun::Preparing && i < 100; ++i)
    autoTick();
  return (currentState == AUTO_MODE || currentState==AUTO_TUNING) && autoRun == AutoRun::Running && !arm.busy();
}
static void autoCounterResetRequiresStoppedAndPreservesSettings() {
  for(unsigned mode=0;mode<3;++mode) {
    setupAutoHarness(-135.14f,mode==1,false);
    if(mode==2)currentState=AUTO_TUNING;
    check(startAutoHarness(),10100);
    sortedParts=8;auto_parts_missed=2;rejectedParts=3;ignoredParts=4;outsidePickupParts=1;
    const uint32_t positions=positionCommands,grips=gripperCommands;
    const uint32_t generation=settingsStore.generation,flashOperations=settingsFlash.operation;
    const float margin=activeTuning.marginMs,tallOffset=activeTuning.tall.partXOffsetMm;
    totalObjects=0;newLatest=true;visionTimestampMs=adapterTime+20;autoTick();
    Serial.clear();autoTick(20,BTN_L3);
    check(autoRun==AutoRun::Running && streamStarted && sortedParts==8 && auto_parts_missed==2 &&
          rejectedParts==3 && ignoredParts==4 && outsidePickupParts==1 &&
          strstr(Serial.text,"Counter reset requires STOPPED"),10101);
    autoTick(20,BTN_Y);fakeAckType=CMD_END_STREAM;Serial.clear();autoTick();
    check(autoRun==AutoRun::Stopped && !streamStarted && strstr(Serial.text,"visionUpdates=1"),10102);
    // A held R3 plus an L3 edge is a chord, not a standalone reset.
    currentControlCMD.buttons=BTN_R3;Serial.clear();autoTick(20,BTN_L3);
    check(sortedParts==8 && !strstr(Serial.text,"Counters reset"),10103);
    currentControlCMD.buttons=0;pendingESPmsg.pending=true;autoTick(20,BTN_L3);
    check(sortedParts==8 && auto_parts_missed==2,10104);
    const float draftMargin=tuningDraft.marginMs;
    pendingESPmsg.pending=false;currentControlCMD.jsLx=1;Serial.clear();autoTick(20,BTN_L3);
    currentControlCMD.jsLx=0;
    check(autoRun==AutoRun::Stopped && !streamStarted && !arm.busy() && sortedParts==0 &&
          auto_parts_missed==0 && rejectedParts==0 && ignoredParts==0 && outsidePickupParts==0 &&
          strstr(Serial.text,"totals sorted=0 missed=0 rejected=0 ignored=0 outsidePickup=0 visionUpdates=0"),10105);
    check(positionCommands==positions && gripperCommands==grips && settingsStore.generation==generation &&
          settingsFlash.operation==flashOperations && activeTuning.marginMs==margin &&
          activeTuning.tall.partXOffsetMm==tallOffset && tuningDraft.marginMs==draftMargin && homographyDone && camera_calib.valid &&
          velocityCalibrated && fabsf(mainTracker.velocity()+135.14f)<.001f,10106);
    if(mode==2) {
      currentControlCMD.buttons=BTN_L3;currentControlCMD.jsLx=1;autoTick();
      check(tuningDraft.marginMs==draftMargin && pickupEditorNeedsNeutral,10107);
      currentControlCMD.buttons=0;autoTick();
      check(tuningDraft.marginMs==draftMargin && pickupEditorNeedsNeutral,10108);
      currentControlCMD.jsLx=0;autoTick();
      check(tuningDraft.marginMs==draftMargin && !pickupEditorNeedsNeutral,10109);
    }
  }
}
static uint32_t feedAutoTrack(float x, float y, uint8_t id, float velocity, int observations=0,
                              bool tall=false) {
  uint32_t firstMs = adapterTime + 100;
  const int samples = observations ? observations : menu_option0 == 1 ?
      int(1 + (CameraTracking::ClassificationConfidence::minimumStableMs + 99) / 100) : 3;
  for (int sample = 0; sample < samples; ++sample) {
    // x denotes corrected robot X. Undo the configured offset in the synthetic camera input.
    latestReceived[0] = calibrationDetection(uint16_t(lroundf(x - mainTracker.cameraXOffsetMm())),
        uint16_t(lroundf(y + velocity * sample * .1f)));
    latestReceived[0].ID = id;
    if(tall) {latestReceived[0].width=20;latestReceived[0].height=40;}
    totalObjects = 1;
    visionTimestampMs = adapterTime + 100;
    newLatest = true;
    autoTick(100);
  }
  return firstMs;
}
static void automaticRealCameraCycles() {
  const float velocities[3] = {-80, -125.71f, -160};
  const float xs[3] = {200, 332, 420};
  for (int bin = 1; bin <= 3; ++bin) {
    setupAutoHarness(velocities[bin - 1]);
    if (bin == 2)
      autoHarnessWristError = -1.02f;
    check(startAutoHarness(), 8120 + bin);
    uint32_t firstMs = feedAutoTrack(xs[bin - 1], 660, partForBin(bin), velocities[bin - 1]);
    check(arm.busy() && autoStage("PLAN"), 8124 + bin);
    bool sawPickup = false, sawWaypoint = false, sawDrop = false, sawReturn = false;
    float pickupQ5 = NAN, measuredQ4 = NAN;
    for (int i = 0; currentState == AUTO_MODE && i < 1000; ++i) {
      if (autoStage("DESCEND")) {
        float pose[6];
        robot_fkine(commanded, params, pose);
        if (!sawPickup) {
          check(hypotf(hypotf(pose[0] - xs[bin - 1], pose[1]),
                       pose[2] - PickMotion::beltContactZ(xs[bin - 1])) <= .5f, 8130 + bin);
          pickupQ5 = commanded[4];
          sawPickup = true;
        }
      }
      if (autoStage("CLOSE"))
        measuredQ4 = armFeedback.q[3];
      if (autoStage("CLASSIFICATION WAYPOINT") && !sawWaypoint) {
        check(fabsf(commanded[3] - measuredQ4) < .001f &&
                  fabsf(commanded[4] - pickupQ5) < .001f, 8134 + bin);
        sawWaypoint = true;
      }
      if (autoStage("CLASSIFICATION POSE") && !sawDrop) {
        float pose[6];
        robot_fkine(commanded, params, pose);
        const float *destination = autoConfig.destinations[bin - 1].pose;
        check(hypotf(hypotf(pose[0] - destination[0], pose[1] - destination[1]),
                     pose[2] - destination[2]) <= .5f &&
                  commanded[0] == 0 && fabsf(pose[3]-destination[3])<2.5f &&
                  fabsf(pose[4]-destination[4])<2.5f && fabsf(pose[5]-destination[5])<2.5f, 8138 + bin);
        sawDrop = true;
      }
      if (autoRun == AutoRun::ReturningHome) {
        sawReturn = true;
        bool zero = true;
        for (float q : commanded) zero &= q == 0;
        check(zero && sortedParts == 1, 8142 + bin);
      }
      if (sawReturn && autoRun == AutoRun::Running && !arm.busy())
        break;
      autoTick();
    }
    bool complete = currentState == AUTO_MODE && sortedParts == 1 && auto_parts_missed == 0 &&
              rejectedParts == 0 && sawPickup && sawWaypoint && sawDrop && sawReturn &&
              autoRun == AutoRun::Running && !arm.busy();
    if (!complete && !test_context[0])
      snprintf(test_context, sizeof(test_context),
               "AUTO bin=%d state=%d run=%d phase=%s sorted=%lu missed=%d rejected=%lu "
               "fault=%s rejection=%s", bin, int(currentState), int(autoRun), arm.phaseName(),
               (unsigned long)sortedParts, auto_parts_missed, (unsigned long)rejectedParts,
               arm.fault(), arm.rejectionReason());
    check(complete, 8146 + bin);
    double expected = 660.0 / -velocities[bin - 1] * 1000 - autoConfig.closingLeadMs;
    check(sawClose && fabs(double(uint32_t(closeTime - firstMs)) - expected) <= 50, 8150 + bin);
    check(positionCommands == 6 && gripperCommands == 4, 8154 + bin);
  }
}
static void automaticStopCompletesHomeBeforeStreamStop() {
  setupAutoHarness();
  check(startAutoHarness(), 8170);
  feedAutoTrack(289, 660, partForBin(1), mainTracker.velocity());
  check(autoUntil("CLASSIFICATION WAYPOINT"), 8171);
  autoTick(20, BTN_Y | BTN_A); // Stop beats A and preserves the assigned part.
  check(stopRequested && !exitRequested && lastCriticalType == CMD_START_STREAM, 8172);
  for (int i = 0; autoRun != AutoRun::ReturningHome && currentState == AUTO_MODE && i < 200; ++i)
    autoTick();
  check(autoRun == AutoRun::ReturningHome && sortedParts == 1 && arm.busy() && streamStarted,
        8173);
  uint32_t before = positionCommands;
  feedAutoTrack(380, 660, partForBin(2), mainTracker.velocity());
  Part observed;
  bool tracked = false;
  for (int i = 0; i < MAX_PARTS; ++i)
    tracked |= mainTracker.candidate(i, observed, adapterTime) && observed.classId == partForBin(2);
  check(tracked && positionCommands == before, 8174);
  for (int i = 0; autoRun != AutoRun::Ending && currentState == AUTO_MODE && i < 200; ++i)
    autoTick();
  check(autoRun == AutoRun::Ending && !arm.busy() && lastCriticalType == CMD_END_STREAM &&
            sortedParts == 1 && positionCommands == before, 8175);
  fakeAckType = CMD_END_STREAM;
  autoTick();
  check(autoRun == AutoRun::Stopped && !streamStarted && currentState == AUTO_MODE, 8176);
  before = criticalCommands;
  autoTick(20, BTN_A | BTN_Y);
  check(autoRun == AutoRun::Stopped && criticalCommands == before, 8177);
  autoTick(20, BTN_L1);
  check(currentState == AUTO_MENU && !arm.busy(), 8178);
}
static void automaticTrackingContinuesDuringReturn() {
  setupAutoHarness();
  check(startAutoHarness(), 8190);
  feedAutoTrack(260, 660, partForBin(1), mainTracker.velocity());
  for (int i = 0; autoRun != AutoRun::ReturningHome && currentState == AUTO_MODE && i < 600; ++i)
    autoTick();
  check(autoRun == AutoRun::ReturningHome && sortedParts == 1, 8191);
  uint32_t before = positionCommands;
  feedAutoTrack(380, 660, partForBin(2), mainTracker.velocity(), 3);
  check(autoRun == AutoRun::ReturningHome && positionCommands == before, 8192);
  // HOME may finish while the label is still pending. Fresh observations must
  // continue until the confirmation span is satisfied before the next pickup.
  feedAutoTrack(380, 660+.3f*mainTracker.velocity(), partForBin(2), mainTracker.velocity(), 4);
  for (int i = 0; autoRun == AutoRun::ReturningHome && i < 100; ++i)
    autoTick();
  check(autoRun == AutoRun::Running && autoStage("PLAN") &&
            fabsf(arm.planReport().pose[0] - 380) < .01f && sortedParts == 1, 8193);
  currentControlCMD.buttons = BTN_B;
  autoTick();
  check(currentState == EMERGENCY_STOP && !arm.busy() && entered_ES &&
            strcmp(Serial1.batch, "S:ESTOP\n") == 0, 8194);
  before = positionCommands;
  currentControlCMD.buttons = 0;
  autoTick();
  check(currentState == EMERGENCY_STOP && positionCommands == before, 8195);
}
static void automaticVisionOnlyAndAdmissionGuards() {
  setupAutoHarness(-125.71f, true, false);
  limits_received = armFeedback.valid = false;
  check(startAutoHarness(), 8210);
  feedAutoTrack(700, 660, 99, mainTracker.velocity());
  check(positionCommands == 0 && gripperCommands == 0 && streamStarted &&
            currentState == AUTO_MODE && autoRun == AutoRun::Running, 8211);
  autoTick(20, BTN_Y);
  check(autoRun == AutoRun::Ending && positionCommands == 0 && gripperCommands == 0, 8212);
  fakeAckType = CMD_END_STREAM;
  autoTick();
  check(autoRun == AutoRun::Stopped && !streamStarted, 8213);

  setupAutoHarness();
  check(startAutoHarness(), 8214);
  uint32_t before = positionCommands;
  feedAutoTrack(332, 660, 99, mainTracker.velocity());
  check(ignoredParts == 1 && !arm.busy() && positionCommands == before, 8215);
  feedAutoTrack(PickMotion::maximumXmm + 20, 660, partForBin(1), mainTracker.velocity());
  check(outsidePickupParts == 1 && ignoredParts == 2 && positionCommands == before, 8216);
  feedAutoTrack(289, 200, partForBin(1), mainTracker.velocity());
  check(auto_parts_missed == 1 && !arm.busy() && positionCommands == before, 8217);
  autoRun = AutoRun::ReturningHome;
  feedAutoTrack(380, 660, partForBin(2), mainTracker.velocity());
  selectNextPart(adapterTime);
  check(positionCommands == before && !arm.busy(), 8218);
}
static void automaticSupervisionAndEarlyStop() {
  setupAutoHarness();
  check(startAutoHarness(), 8230);
  feedAutoTrack(289, 660, partForBin(1), mainTracker.velocity());
  check(autoUntil("APPROACH"), 8231);
  uint32_t before = positionCommands;
  autoTick(autoConfig.feedbackTimeoutMs + 1, 0, false, false);
  check(currentState == EMERGENCY_STOP && !arm.busy() && positionCommands == before, 8232);

  setupAutoHarness();
  autoTick(0);
  fakeAckType = CMD_END_STREAM;
  autoTick();
  autoTick(20, BTN_A);
  autoTick(20, BTN_L1); // Cancel startup before stream-start acknowledgement.
  fakeAckType = CMD_START_STREAM;
  autoTick();
  check(autoRun == AutoRun::Ending && positionCommands == 0 && gripperCommands == 0, 8233);
  fakeAckType = CMD_END_STREAM;
  autoTick();
  check(currentState == AUTO_MENU && autoRun == AutoRun::Stopped, 8234);
}

static void planningConsumesItsAdmissionBudgetOnlyOnce() {
  Fixture f;
  f.config = TrajectoryTest::configuration(f.config, 332, f.params);
  f.config.pickUpMarginMs = 1000;
  check(f.pick(), 8250); // Initially 3500 ms remain before close.
  f.tick(600, false); // Planning consumed 600 ms; remaining delay is now 2900.
  uint32_t id;
  check(f.arm.busy() && f.arm.takeOutcome(id) == PickOutcome::None, 8251);
  check(f.finish() == PickOutcome::Sorted, 8252);
  Fixture expired;
  check(expired.pick(), 8253);
  expired.tick(3501, false);
  check(expired.finish() == PickOutcome::Missed &&
            positionCommands == 1 && gripperCommands == 0, 8254);
}

static void automaticMissedPickReturnsHomeAndControllerLossStopsReturn() {
  setupAutoHarness();
  check(startAutoHarness(), 8270);
  feedAutoTrack(289, 660, partForBin(1), mainTracker.velocity());
  check(autoUntil("APPROACH"), 8271);
  // Fresh feedback from an arm that is actually still at HOME, past deadline.
  memset(armFeedback.q, 0, sizeof(armFeedback.q));
  autoTick(5000, 0, false);
  check(autoStage("MISSED-PICK RECOVERY") && !sawClose, 8272);
  for (int i = 0; autoRun != AutoRun::ReturningHome && currentState == AUTO_MODE && i < 100; ++i)
    autoTick();
  check(autoRun == AutoRun::Running && auto_parts_missed == 1 && sortedParts == 0 &&
            !sawClose, 8273);
  for (int i = 0; autoRun == AutoRun::ReturningHome && currentState == AUTO_MODE && i < 100; ++i)
    autoTick();
  check(currentState == AUTO_MODE && autoRun == AutoRun::Running && !arm.busy(), 8274);

  // An independent controller-loss event still stops an active HOME return.
  setupAutoHarness();
  check(startAutoHarness(), 8275);
  feedAutoTrack(260, 660, partForBin(1), mainTracker.velocity());
  for (int i = 0; autoRun != AutoRun::ReturningHome && currentState == AUTO_MODE && i < 600; ++i)
    autoTick();
  check(autoRun == AutoRun::ReturningHome && arm.busy(), 8276);
  uint32_t before = positionCommands;
  udpControllerStable = false;
  autoTick();
  check(currentState == EMERGENCY_STOP && !arm.busy() && positionCommands == before &&
            strcmp(Serial1.batch, "S:ESTOP\n") == 0, 8277);
}

static void partMeasurementsNeverChangeCalibratedSpeed() {
  const calibH c = identityCalibration();
  BeltTracker tracker(0, fixtureVelocityPixels);
  tracker.setVelocity(-125.71f);
  for (int i = 0; i < 22; ++i) {
    objectVision d = calibrationDetection(324, 660-16*i);
    tracker.updateFrame(&d, 1, 1000+100*i, c);
  }
  Part p;
  check(tracker.calibrated() && tracker.velocity() == -125.71f &&
        tracker.candidate(0, p, 3100), 8290);
  const float y = p.y;
  tracker.tick(4100);
  check(tracker.candidate(0, p, 4100) && fabsf(p.y-(y-125.71f)) < .001f, 8291);
  tracker.clear();
  check(tracker.calibrated() && tracker.velocity() == -125.71f, 8292);
}

static void newPartsWhileSortingDoNotReplaceTheAssignedSnapshot() {
  setupAutoHarness(-100);
  check(startAutoHarness(), 8610);
  feedAutoTrack(289, 660, partForBin(1), -100);
  check(autoUntil("CLASSIFICATION WAYPOINT"), 8611); // Closing has completed.
  for (int i=0; i<28 && currentState==AUTO_MODE; ++i) {
    const float bias = i <= 9 ? -3.5f*i : i < 18 ? -3.5f*(18-i) : 0;
    latestReceived[0] = calibrationDetection(
      uint16_t(lroundf(380-CameraTracking::partXOffsetMm)),
      uint16_t(lroundf(660-10*i+bias)));
    latestReceived[0].ID = partForBin(2);
    totalObjects = 1; visionTimestampMs = adapterTime+100; newLatest = true;
    autoTick(100);
  }
  check(currentState==AUTO_MODE && velocityCalibrated && mainTracker.velocity()==-100 &&
        sortedParts>=1, 8612);
  // Different apparent speeds from later parts must not alter calibration or stop sorting.
  setupAutoHarness(-125.71f);
  check(startAutoHarness(), 8613);
  feedAutoTrack(289, 660, partForBin(1), -125.71f);
  check(autoUntil("CLASSIFICATION WAYPOINT"), 8614);
  Serial.clear();
  for (int i=0; i<20 && currentState==AUTO_MODE; ++i) {
    latestReceived[0] = calibrationDetection(
      uint16_t(lroundf(380-CameraTracking::partXOffsetMm)), 660-16*i);
    latestReceived[0].ID = partForBin(2);
    totalObjects = 1; visionTimestampMs = adapterTime+100; newLatest = true;
    autoTick(100);
  }
  check(currentState==AUTO_MODE && velocityCalibrated &&
        mainTracker.velocity()==-125.71f && sortedParts>=1, 8615);
  check(!strstr(Serial.text,"[BELT SPEED] track=") && !strstr(Serial.text,"[ESTOP]"), 8616);
}

static void feedClassObservation(unsigned step, uint8_t id) {
  latestReceived[0]=calibrationDetection(
      uint16_t(lroundf(332-CameraTracking::partXOffsetMm)),660-13*step);
  latestReceived[0].ID=id; totalObjects=1;
  visionTimestampMs=adapterTime+100; newLatest=true;
  autoTick(100);
}
static void transientFirstLabelDoesNotChooseTheBin() {
  const uint8_t initial[]={partForBin(3),partForBin(3),99};
  const uint8_t bins[]={1,2,1};
  const unsigned confirmationStep=5+(CameraTracking::ClassificationConfidence::minimumStableMs+99)/100;
  for (unsigned example=0;example<3;++example) {
    setupAutoHarness(-128.86f); check(startAutoHarness(),9100);
    const uint32_t before=positionCommands;
    Serial.clear();
    for (unsigned i=0;i<=confirmationStep;++i) {
      feedClassObservation(i,i<5?initial[example]:partForBin(bins[example]));
      if (i<confirmationStep)
        check(!arm.busy() && positionCommands==before && ignoredParts==0 &&
              auto_parts_missed==0,9101);
    }
    char assigned[40]; snprintf(assigned,sizeof(assigned),"part ID=%u bin=%u",
                                unsigned(partForBin(bins[example])),unsigned(bins[example]));
    check(arm.busy() && autoStage("PLAN") && strstr(Serial.text,assigned) &&
          strstr(Serial.text,"Waiting for classification"),9102);
    // Classification remains fixed after reservation, as requested. Even a
    // later long run of another label must not change the sorting destination.
    for (unsigned i=confirmationStep+1;i<confirmationStep+9;++i) feedClassObservation(i,partForBin(3));
    check(autoUntil("CLASSIFICATION POSE"),9103);
    float tcp[6]; robot_fkine(commanded,params,tcp);
    const float *destination=autoConfig.destinations[bins[example]-1].pose;
    check(hypotf(hypotf(tcp[0]-destination[0],tcp[1]-destination[1]),
                 tcp[2]-destination[2])<=.5f,9104);
  }
  setupAutoHarness(-128.86f,false,false); check(startAutoHarness(),9105);
  for (unsigned i=0;i<3;++i) feedClassObservation(i,99);
  check(arm.busy() && autoStage("PLAN"),9106); // ID-ignorant admission is unchanged.
}
static void classificationNeedsObservedSpanNotHeldPackets() {
  BeltTracker tracker(0, fixtureVelocityPixels); tracker.setVelocity(-100);
  const calibH c=identityCalibration();
  objectVision d=calibrationDetection(324,420); d.ID=partForBin(1);
  for (unsigned ms=0;ms<=1200;ms+=20) {
    d.width=uint16_t(10+ms%3); tracker.updateFrame(&d,1,1000+ms,c);
  }
  Part p;
  check(tracker.candidate(0,p,2200) && p.hits>3 && p.classVotes==1 &&
        p.classLastMs==p.classSinceMs && !partClassificationStable(p),9120);
  AutoPick::Request request; const AutoConfig base=automaticConfiguration();
  check(AutoPick::evaluate(p,-100,base,true,2200,request)==
          AutoPick::Admission::AwaitingClassification &&
        AutoPick::evaluate(p,-100,base,false,2200,request)==AutoPick::Admission::Eligible,9121);
  tracker.clear();
  d=calibrationDetection(324,420); d.ID=partForBin(1);
  tracker.updateFrame(&d,1,1000,c);
  d.yCenter=416; tracker.updateFrame(&d,1,1040,c);
  tracker.updateFrame(&d,1,1100,c); // Late held copy of the throttled position.
  check(tracker.candidate(0,p,1100) && p.classVotes==1,9122);
  d.yCenter=384; tracker.updateFrame(&d,1,1360,c);
  d.yCenter=352; tracker.updateFrame(&d,1,1680,c);
  check(tracker.candidate(0,p,1680) && partClassificationStable(p) &&
        uint32_t(p.classLastMs-p.classSinceMs)==680,9123);
  d.yCenter=260; tracker.updateFrame(&d,1,2600,c);
  check(tracker.candidate(0,p,2600) && p.classVotes==1 && !partClassificationStable(p),9124);
  d.ID=partForBin(3); d.yCenter=248; tracker.updateFrame(&d,1,2720,c);
  check(tracker.candidate(0,p,2720) && p.classId==partForBin(1) &&
        p.pendingClass==partForBin(3) && !partClassificationStable(p),9125);
  tracker.clear();
  for (unsigned ms=0;ms<=600;ms+=300) {
    d.yCenter=uint16_t(660-ms/10);
    tracker.updateFrame(&d,1,uint32_t(0xffffff00U+ms),c);
  }
  check(tracker.candidate(0,p,uint32_t(0xffffff00U+600)) && partClassificationStable(p),9126);
}
static void uncertainClassExpiresWithoutStartingTheArm() {
  setupAutoHarness(-128.86f); check(startAutoHarness(),9140);
  const uint32_t before=positionCommands; Serial.clear();
  for (unsigned i=0;i<3;++i) feedClassObservation(i,partForBin(3));
  check(!arm.busy(),9141);
  for (unsigned i=0;i<220 && !auto_parts_missed;++i) autoTick();
  check(auto_parts_missed==1 && ignoredParts==0 && rejectedParts==0 &&
        positionCommands==before && !arm.busy() &&
        strstr(Serial.text,"classification=pending"),9142);
  for (unsigned i=0;i<20;++i) autoTick();
  check(auto_parts_missed==1,9143);
}

static void automaticReadinessReportsActualBlocker() {
  setupAutoHarness();
  lastState = currentState; // Already synchronized and stopped at the AUTO screen.
  const float previousMax = LIMIT_GRIPPER_MAX;
  LIMIT_GRIPPER_MAX = 160;
  autoConfig.closeAngle = 165;
  check(!autoReady() && strcmp(autoReadinessIssue(),
        "closeAngle outside Nucleo gripper limits") == 0, 8300);
  // A must neither start the camera stream nor send arm/gripper commands.
  serviceAutomaticMode(adapterTime, BTN_A);
  check(autoRun == AutoRun::Stopped && criticalCommands == 0 &&
        positionCommands == 0 && gripperCommands == 0, 8301);
  autoConfig.closeAngle = 160;
  autoConfig.closingLeadMs = 450; // User's timing adjustment remains valid.
  check(autoReady() && autoReadinessIssue() == nullptr, 8302);
  serviceAutomaticMode(adapterTime, BTN_A);
  check(autoRun == AutoRun::Starting && criticalCommands == 1 &&
        lastCriticalType == CMD_START_STREAM, 8311);
  autoConfig.openAngle = -1;
  check(strcmp(autoReadinessIssue(), "openAngle outside Nucleo gripper limits") == 0, 8303);
  autoConfig.openAngle = 0;
  limits_received = false;
  check(strcmp(autoReadinessIssue(), "Joint/gripper limits not received from Nucleo") == 0, 8304);
  limits_received = true;
  armFeedback.valid = false;
  check(strcmp(autoReadinessIssue(), "Nucleo feedback invalid or stale") == 0, 8305);
  armFeedback.valid = true;
  armFeedback.timestampMs = adapterTime - autoConfig.feedbackTimeoutMs - 1;
  check(!autoReady() && strcmp(autoReadinessIssue(), "Nucleo feedback invalid or stale") == 0, 8306);
  armFeedback.timestampMs = adapterTime;
  homographyDone = false;
  check(strcmp(autoReadinessIssue(), "Camera calibration missing or invalid") == 0, 8307);
  homographyDone = true;
  velocityCalibrated = false;
  check(strcmp(autoReadinessIssue(), "Belt velocity calibration missing or invalid") == 0, 8308);
  velocityCalibrated = true;
  cameraOffline = true;
  check(strcmp(autoReadinessIssue(), "Camera unavailable") == 0, 8309);
  cameraOffline = false;
  autoConfig.closeAngle = 165;
  menu_option2 = 1; // Vision-only still bypasses arm configuration.
  check(autoReady(), 8310);
  LIMIT_GRIPPER_MAX = previousMax;
}

static void cameraPartOffsetPrecedesTrackingAndAdmission() {
  const calibH c = identityCalibration();
  const AutoConfig base = automaticConfiguration();
  const float offsets[] = {0.0f, 20.0f, -20.0f};
  for (float offset : offsets) {
    BeltTracker tracker(offset, fixtureVelocityPixels);
    BeltCalibrator calibration(offset, fixtureVelocityPixels);
    tracker.setVelocity(-100);
    calibration.reset();
    for (int sample = 0; sample < 12; ++sample) {
      objectVision d = calibrationDetection(300, 660 - 10 * sample);
      tracker.updateFrame(&d, 1, 1000 + 100 * sample, c);
      check(calibration.add(d, 1000 + 100 * sample, c), 8400);
    }
    Part p;
    check(tracker.candidate(0, p, 2100) && p.hits == 12 &&
          fabsf(p.x - (300 + offset)) < .001f && fabsf(p.y - 550) < .001f &&
          tracker.velocity() == -100, 8401);
    Part unused;
    check(!tracker.candidate(1, unused, 2100), 8402); // No duplicate track on repeated correction.
    AutoPick::Request request;
    check(AutoPick::evaluate(p, -100, base, false, 2100, request) ==
          AutoPick::Admission::Eligible && request.part.x == p.x &&
          fabsf(request.config.pickupZ - PickMotion::beltContactZ(300 + offset)) < .001f &&
          request.closingDelayMs == 5500 - base.closingLeadMs, 8403);
    float velocity;
    BeltCalibrationReport r = calibration.report();
    check(calibration.finish(velocity) && fabsf(velocity + 100) < .001f &&
          fabsf(r.worldX - (300 + offset)) < .001f && fabsf(r.worldY - 550) < .001f &&
          fabsf(r.deltaXmm) < .001f, 8404);
    tracker.clear();
    tracker.setVelocity(-100);
    calibration.reset();
    for (int sample = 0; sample < 3; ++sample) {
      objectVision d = calibrationDetection(300, 660 - 10 * sample);
      tracker.updateFrame(&d, 1, 3000 + 100 * sample, c);
      calibration.add(d, 3000 + 100 * sample, c);
    }
    check(tracker.candidate(0, p, 3200) && fabsf(p.x - (300 + offset)) < .001f &&
          fabsf(calibration.report().worldX - (300 + offset)) < .001f, 8405);
  }
  // Bounds apply to the corrected pickup X, even if the raw X was in range.
  const float boundaryOffsets[] = {20.0f, -20.0f};
  for (float offset : boundaryOffsets) {
    BeltTracker tracker(offset, fixtureVelocityPixels);
    tracker.setVelocity(-100);
    const uint16_t rawX = offset > 0 ? PickMotion::maximumXmm : PickMotion::minimumXmm;
    for (int sample = 0; sample < 3; ++sample) {
      objectVision d = calibrationDetection(rawX, 660 - 10 * sample);
      tracker.updateFrame(&d, 1, 1000 + 100 * sample, c);
    }
    Part p;
    AutoPick::Request request;
    check(tracker.candidate(0, p, 1200) &&
          AutoPick::evaluate(p, -100, base, false, 1200, request) ==
          AutoPick::Admission::OutsidePickupRange, 8406);
  }
  BeltTracker invalid(NAN, fixtureVelocityPixels);
  invalid.setVelocity(-100);
  objectVision d = calibrationDetection(300, 660);
  for (int i = 0; i < 3; ++i)
    invalid.updateFrame(&d, 1, 1000, c);
  Part p;
  check(!invalid.candidate(0, p, 1000), 8407);
  BeltCalibrator invalidCalibration(NAN, fixtureVelocityPixels);
  check(!invalidCalibration.add(d, 1000, c) &&
        invalidCalibration.report().issue == BeltCalibrationIssue::NonfiniteProjection, 8408);
}

static void visionDiagnosticsWithoutArmMotion() {
  setupAutoHarness(-100, true, true);
  limits_received = armFeedback.valid = false;
  Serial.clear();
  check(startAutoHarness(), 8500);
  check(strstr(Serial.text, "ONLY VISION; press A") &&
        strstr(Serial.text, "START_STREAM requested") &&
        strstr(Serial.text, "START_STREAM acknowledged") &&
        strstr(Serial.text, "Stage: VISION TRACKING"), 8501);
  Serial.clear();
  autoTick(600, 0, false, false);
  check(strstr(Serial.text, "No DATA_OBJECTS since start"), 8502);
  const uint8_t ids[3] = {2, 5, 8};
  for (int frame = 0; frame < 3; ++frame) {
    totalObjects = 3;
    for (int i = 0; i < 3; ++i) {
      latestReceived[i] = calibrationDetection(280 + 50 * i, 650 - 10 * frame);
      latestReceived[i].ID = ids[i];
    }
    newLatest = true;
    visionTimestampMs = adapterTime + 100;
    autoTick(100, 0, false, false);
  }
  Serial.clear();
  autoTick(300, 0, false, false);
  check(strstr(Serial.text, "updates=3") && strstr(Serial.text, "IDs=[2,5,8]") &&
        strstr(Serial.text, "candidates=3"), 8503);
  check(currentState == AUTO_MODE && streamStarted && autoRun == AutoRun::Running &&
        positionCommands == 0 && gripperCommands == 0 && !arm.busy(), 8504);
  Serial.clear();
  autoTick(1100, 0, false, false);
  check(strstr(Serial.text, "no recent DATA_OBJECTS") &&
        !strstr(Serial.text, "world="), 8505);
  autoTick(20, BTN_Y, false, false);
  fakeAckType = CMD_END_STREAM;
  autoTick(20, 0, false, false);
  check(autoRun == AutoRun::Stopped && !streamStarted &&
        strstr(Serial.text, "visionUpdates=3") &&
        strstr(Serial.text, "Stage: VISION STOPPED"), 8506);
  Serial.clear();
  autoTick(20, BTN_A, false, false);
  fakeAckType = CMD_START_STREAM;
  autoTick(20, 0, false, false);
  Serial.clear();
  autoTick(600, 0, false, false);
  check(strstr(Serial.text, "No DATA_OBJECTS since start"), 8507);
  // Empty frames and all ten uint8_t IDs must format without truncating the list.
  totalObjects = 0;
  newLatest = true;
  visionTimestampMs = adapterTime + 600;
  Serial.clear();
  autoTick(600, 0, false, false);
  check(strstr(Serial.text, "boxes=0 candidates=0"), 8508);
  totalObjects = 10;
  for (int i = 0; i < 10; ++i) {
    latestReceived[i] = calibrationDetection(280 + i, 650);
    latestReceived[i].ID = 255;
  }
  newLatest = true;
  visionTimestampMs = adapterTime + 600;
  Serial.clear();
  autoTick(600, 0, false, false);
  check(strstr(Serial.text, "IDs=[255,255,255,255,255,255,255,255,255,255]"), 8509);
  check(positionCommands == 0 && gripperCommands == 0, 8510);
}

static void diagnosticQueueSurvivesBusyAndShortWrites() {
  struct Port {
    int space = 0;
    size_t limit = 100, largest = 0, used = 0;
    char text[2048]{};
    int availableForWrite() { return space; }
    size_t write(const uint8_t *data, size_t count) {
      if (count > largest) largest = count;
      if (count > limit) count = limit;
      memcpy(text + used, data, count);
      used += count;
      text[used] = '\0';
      return count;
    }
  } port;
  DiagnosticQueue queue;
  const char *line = "[CLASS TX] model=131 box=1/1 cameraID=2 name=\"Pieza_02\" sentID=2\n";
  check(queue.push(line), 8520);
  queue.drain(port); // No free UART space: retain the complete message.
  check(queue.pending() == strlen(line) && port.used == 0, 8521);
  port.space = 7; // Never enough space for the whole line.
  port.limit = 3; // Also simulate partial writes.
  for (int i = 0; i < 100 && queue.pending(); ++i) queue.drain(port);
  check(!queue.pending() && strcmp(port.text, line) == 0 && port.largest <= 7, 8522);
  port.space = 128;
  port.limit = 100;
  port.used = 0;
  port.largest = 0;
  for (int i = 0; i < 14; ++i) {
    check(queue.push(line), 8523);
    for (int j = 0; j < 10 && queue.pending(); ++j) queue.drain(port);
  }
  check(port.used == 14 * strlen(line) && port.largest <= 32, 8524);
  // Full queue drops new text, without corrupting text already queued.
  char full[769];
  memset(full, 'X', 768); full[768] = '\0';
  check(queue.push(full) && !queue.push("Y") && queue.dropped() == 1, 8525);
  port.used = 0;
  for (int i = 0; i < 100 && queue.pending(); ++i) queue.drain(port);
  check(port.used == 768 && !queue.pending() && memcmp(port.text, full, 768) == 0, 8526);
}

static char jointTestMessage[128];
static bool jointTestWriteFails;
static bool sendJointTestMessage(const char *b) {
  snprintf(jointTestMessage,sizeof(jointTestMessage),"%s",b); return !jointTestWriteFails;
}
static void logJointTest(const char *) {}
static void jointTestTick(JointMotionTest &test,Fixture &f,uint32_t ms=20,unsigned buttons=0,
                          bool ack=true,bool follow=true) {
  f.now+=ms; ioTime=f.now;
  if (follow) memcpy(f.feedback.q,commanded,sizeof(commanded));
  ++f.feedback.sequence; f.feedback.timestampMs=f.now;
  if (ack && test.runId()) {
    char b[64]; snprintf(b,sizeof(b),"D:%lu,%u,20,152",(unsigned long)test.runId(),
      unsigned(test.state()!=JointMotionTest::State::Ending));
    test.receive(b,f.now);
  }
  test.tick(f.feedback,f.now,buttons,91);
}
static void jointMotionRepeatedRunAndLoggingHandshake() {
  Fixture f; JointMotionTest test; test.bind(sendJointTestMessage,logJointTest);
  jointTestWriteFails=false; test.enter(f.arm,f.params,f.config,2);
  check(!f.arm.busy() && positionCommands==0 && test.state()==JointMotionTest::State::Ready,8700);
  jointTestTick(test,f,0,JointMotionTest::Start,false);
  check(strstr(jointTestMessage,"D:START,91") && !f.arm.busy() && positionCommands==0,8701);
  test.receive("D:92,1,20,152",f.now); jointTestTick(test,f,100,0,false);
  check(!f.arm.busy(),8702); // A different run cannot authorise motion.
  for (int i=0;i<200 && test.state()!=JointMotionTest::State::Baseline;++i) jointTestTick(test,f);
  check(test.state()==JointMotionTest::State::Baseline && test.cycle()==0,8703);
  const uint32_t before=positionCommands;
  jointTestTick(test,f,1000,JointMotionTest::Pause);
  jointTestTick(test,f,10000);
  check(test.state()==JointMotionTest::State::Baseline && positionCommands==before,8704);
  jointTestTick(test,f,20,JointMotionTest::Start);
  for (int i=0;i<1800 && !test.fault() && test.state()!=JointMotionTest::State::Finished;++i)
    jointTestTick(test,f);
  check(!test.fault() && test.state()==JointMotionTest::State::Finished && test.cycle()==2 &&
        !f.arm.busy() && !test.active(),8705);
  for (int i=0;i<5;++i) check(commanded[i]==0,8706+i);
  const uint32_t done=positionCommands;
  jointTestTick(test,f,10000);
  check(positionCommands==done && test.cycle()==2,8711);
  jointTestTick(test,f,0,JointMotionTest::Cancel);
  check(test.exitRequested() && positionCommands==done,8712);
}
static void jointMotionCancelAndFailures() {
  Fixture f; JointMotionTest test; test.bind(sendJointTestMessage,logJointTest);
  test.enter(f.arm,f.params,f.config,1);
  jointTestTick(test,f,0,JointMotionTest::Start,false);
  jointTestTick(test,f,1501,0,false);
  check(test.fault() && !f.arm.busy() && positionCommands==0,8720);
  Fixture cancel; test.enter(cancel.arm,cancel.params,cancel.config,1);
  jointTestTick(test,cancel,0,JointMotionTest::Start,false);
  for (int i=0;i<500 && test.state()!=JointMotionTest::State::Picking;++i) jointTestTick(test,cancel);
  check(test.state()==JointMotionTest::State::Picking,8721);
  jointTestTick(test,cancel,20,JointMotionTest::Pause);
  for (int i=0;i<100 && !cancel.arm.paused();++i) jointTestTick(test,cancel);
  check(cancel.arm.paused(),8722);
  jointTestTick(test,cancel,20,JointMotionTest::Cancel);
  for (int i=0;i<100 && !test.exitRequested() && !test.fault();++i) jointTestTick(test,cancel);
  check(test.exitRequested() && !test.fault() && !cancel.arm.busy() && !test.active(),8723);
  Fixture drift; test.enter(drift.arm,drift.params,drift.config,1);
  jointTestTick(test,drift,0,JointMotionTest::Start,false);
  for (int i=0;i<100 && test.state()!=JointMotionTest::State::Baseline;++i) jointTestTick(test,drift);
  drift.feedback.q[0]=4;
  jointTestTick(test,drift,20,0,true,false);
  check(test.fault() && strstr(test.fault(),"drift"),8724);
  Fixture missing; test.enter(missing.arm,missing.params,missing.config,1);
  jointTestTick(test,missing,0,JointMotionTest::Start,false);
  for (int i=0;i<100 && test.state()!=JointMotionTest::State::Baseline;++i) jointTestTick(test,missing);
  test.tick(missing.feedback,missing.now+500,0,91);
  check(test.fault() && strstr(test.fault(),"stale"),8725);
}

static void velocityRegionFiltersWholeBoxes() {
  const auto &r = CameraTracking::VelocityRegion::rectangle;
  check(r.containsBox(234, 58, 20, 20) && r.containsBox(406, 422, 20, 20), 8800);
  check(!r.containsBox(233, 58, 20, 20) && !r.containsBox(234, 57, 20, 20) &&
        !r.containsBox(407, 422, 20, 20) && !r.containsBox(406, 423, 20, 20), 8801);
  check(!r.containsBox(234, 58, 21, 20) && !r.containsBox(406, 422, 20, 21) &&
        !r.containsBox(324, 240, 0, 20) && !r.containsBox(0, 240, 20, 20) &&
        !r.containsBox(65535, 65535, 65535, 65535), 8802);
  BeltCalibrator b(0);
  const calibH c = identityCalibration();
  check(b.add(calibrationDetection(324, 460), 900, c) && !b.seen() &&
        !b.failed() && b.report().outsideRegion == 1, 8803);
  for (int i = 0; i < 13; ++i)
    check(b.add(calibrationDetection(324, 420-10*i), 1000+100*i, c), 8804);
  const uint32_t accepted = b.lastSampleMs();
  // Centre is inside, but one side of this deformed box is outside the ROI.
  objectVision clipped = calibrationDetection(324, 240); clipped.width = 600;
  check(b.add(clipped, 2300, c) && b.add(calibrationDetection(324, 20), 2400, c), 8805);
  float v = 0;
  check(b.samples() == 13 && b.lastSampleMs() == accepted &&
        b.report().outsideRegion == 3 && b.finish(v) && fabsf(v+100) < .001f &&
        b.report().spanMs == 1200 && b.report().worldY == 300, 8806);
  b.reset();
  check(!b.seen() && b.report().outsideRegion == 0, 8807);


}

static void algorithmSelectionReusesOnlyConfirmedCurrentModel() {
  setupAutoHarness();
  cameraAlgorithm.invalidate();
  requestedTagsAlgo = algoSwitched = invalidAlgo = switchAcked = false;
  requestedAlgoVelocity = true;
  cameraOffline = communicationError = false;
  fakeAckType = 0; pendingESPmsg.pending = false; menu_option1 = 0;
  currentState = WAITING_ALGO_SWITCH; lastState = CALIBRATION_MENU;
  uint32_t before = criticalCommands;
  serviceAlgorithmSelection();
  check(currentState == WAITING_ALGO_SWITCH && criticalCommands == before+1 &&
        lastCriticalParam == APRIL_TAG_ALGO_ID, 8830);
  cameraAlgorithm.acknowledge(); fakeAckType = CMD_SWITCH_ALGO;
  serviceAlgorithmSelection();
  check(currentState == WAITING_ALGO_SWITCH && !cameraAlgorithm.matches(APRIL_TAG_ALGO_ID), 8831);
  cameraAlgorithm.switched(); algoSwitched = true;
  serviceAlgorithmSelection();
  check(currentState == VELOCITY_CALIBRATION && cameraAlgorithm.matches(APRIL_TAG_ALGO_ID), 8832);
  requestedAlgoVelocity = false; requestedTagsAlgo = true;
  currentState = WAITING_ALGO_SWITCH; lastState = CALIBRATION_MENU;
  before = criticalCommands; newLatest = true;
  serviceAlgorithmSelection();
  check(currentState == CAMERA_CALIBRATION && criticalCommands == before && !newLatest, 8833);
  requestedAlgoVelocity = true; requestedTagsAlgo = false;
  currentState = WAITING_ALGO_SWITCH; lastState = CALIBRATION_MENU;
  serviceAlgorithmSelection();
  check(currentState == VELOCITY_CALIBRATION && criticalCommands == before, 8834);
  requestedAlgoVelocity = false;
  currentState = WAITING_ALGO_SWITCH; lastState = AUTO_MENU;
  serviceAlgorithmSelection();
  check(currentState == WAITING_ALGO_SWITCH && lastCriticalParam == CUSTOM_MODEL1_ID &&
        !cameraAlgorithm.matches(APRIL_TAG_ALGO_ID), 8835);
  cameraAlgorithm.switched(); algoSwitched = true;
  serviceAlgorithmSelection();
  check(currentState == WAITING_ALGO_SWITCH, 8836);
  cameraAlgorithm.acknowledge(); fakeAckType = CMD_SWITCH_ALGO;
  serviceAlgorithmSelection();
  check(currentState == AUTO_MODE && cameraAlgorithm.matches(CUSTOM_MODEL1_ID), 8837);
  currentState = WAITING_ALGO_SWITCH; lastState = AUTO_MENU; before = criticalCommands;
  serviceAlgorithmSelection();
  check(currentState == AUTO_MODE && criticalCommands == before, 8838);
  menu_option1 = 1; currentState = WAITING_ALGO_SWITCH; lastState = AUTO_MENU;
  serviceAlgorithmSelection();
  check(currentState == WAITING_ALGO_SWITCH && lastCriticalParam == CUSTOM_MODEL2_ID, 8839);
  cameraAlgorithm.acknowledge(); cameraAlgorithm.switched();
  fakeAckType = CMD_SWITCH_ALGO; algoSwitched = true;
  serviceAlgorithmSelection();
  check(currentState == AUTO_MODE, 8840);
  requestedAlgoVelocity = true;
  currentState = WAITING_ALGO_SWITCH; lastState = CALIBRATION_MENU;
  serviceAlgorithmSelection();
  adapterTime += 12001;
  serviceAlgorithmSelection();
  check(currentState == EMERGENCY_STOP && !cameraAlgorithm.matches(CUSTOM_MODEL2_ID) &&
        !cameraAlgorithm.matches(APRIL_TAG_ALGO_ID), 8841);
  cameraAlgorithm.switched(); cameraAlgorithm.acknowledge();
  check(!cameraAlgorithm.matches(APRIL_TAG_ALGO_ID), 8842);
}

static void fillCornerTags(objectVision *tags) {
  const uint16_t x[] = {80, 560, 80, 560}, y[] = {80, 80, 400, 400};
  for (uint8_t i = 0; i < 4; ++i) {
    tags[i] = calibrationDetection(x[i], y[i]); tags[i].ID = i;
  }
}
static void sparseMovingTagIgnoresDuplicateCornerIds() {
  const calibH c = identityCalibration();
  BeltCalibrator b;
  objectVision tags[5]; fillCornerTags(tags);
  check(b.addFrame(tags,4,900,c) && !b.seen() && b.report().outsideRegion==4, 8900);
  const uint32_t times[] = {1000,1400,1900,2300};
  for (uint8_t id = 0; id <= 3; ++id) {
    b.reset();
    for (unsigned i=0;i<4;++i) {
      tags[4]=calibrationDetection(324,420-(times[i]-1000)/10); tags[4].ID=id;
      check(b.addFrame(tags,5,times[i],c),8901);
      check(b.addFrame(tags,4,times[i]+100,c) && b.lastSampleMs()==times[i],8902);
    }
    float v=0;
    check(b.finish(v) && fabsf(v+100)<.001f && b.samples()==4 &&
          b.report().spanMs==1300 && b.report().tagLocked && b.report().tagId==id &&
          b.report().outsideRegion==32 && fabsf(b.report().worldX-324)<.001f,8903);
  }
  b.reset();
  tags[4]=calibrationDetection(324,420); tags[4].ID=0;
  b.addFrame(tags,5,1000,c);
  tags[4].ID=3;
  check(!b.addFrame(tags,5,1100,c) && b.report().issue==BeltCalibrationIssue::TagChanged,8904);
  b.reset();
  objectVision two[]={calibrationDetection(324,400),calibrationDetection(350,390)};
  two[1].ID=two[0].ID; // Same physical ID at two locations is still ambiguous.
  check(!b.addFrame(two,2,1000,c) && b.report().issue==BeltCalibrationIssue::MultipleTags &&
        b.samples()==0,8905);
  b.reset();
  two[0].ID=8;
  check(b.addFrame(two,1,1000,c) && !b.seen() && b.report().ignoredIds==1,8906);
  check(b.addFrame(nullptr,0,1100,c) && !b.failed(),8907);
  check(!b.addFrame(nullptr,1,1200,c) && b.report().issue==BeltCalibrationIssue::InvalidFrame,8908);
  b.reset();
  for (unsigned i=0;i<3;++i) b.add(calibrationDetection(324,420-40*i),1000+400*i,c);
  float v=0;
  check(!b.finish(v) && b.report().issue==BeltCalibrationIssue::TooFewSamples,8909);
  b.add(calibrationDetection(324,300),2200,c);
  check(b.finish(v) && fabsf(v+100)<.001f,8910);
}

static void movingTagStreamPreservesAllDetections() {
  struct TagBox { int16_t xCenter,yCenter,width,height; char content[16]; };
  struct Frame { TagBox boxes[10]{}; uint8_t count=0; } frame;
  frame.count=5;
  for (unsigned i=0;i<5;++i) {
    frame.boxes[i].xCenter=i==4?324:(i%2?560:80);
    frame.boxes[i].yCenter=i==4?400:80;
    frame.boxes[i].width=frame.boxes[i].height=20;
    snprintf(frame.boxes[i].content,16,"%u",i%4);
  }
  VisionPacket packet{};
  packAprilTags(frame,packet);
  check(packet.vision.amountoObjects==5 && packet.vision.objList[4].ID==0 &&
        packet.vision.objList[4].xCenter==324,8920);
  frame.boxes[0]=frame.boxes[4]; frame.count=1;
  packAprilTags(frame,packet);
  check(packet.vision.amountoObjects==1 && packet.vision.objList[0].ID==0,8921);
  strcpy(frame.boxes[0].content,"not a tag");
  packAprilTags(frame,packet);
  check(packet.vision.amountoObjects==0,8922);
  uint8_t id=99;
  check(parseAprilTagId("0",id) && id==0 && parseAprilTagId(" 255 ",id) && id==255 &&
        !parseAprilTagId("256",id) && !parseAprilTagId("-1",id) &&
        !parseAprilTagId("2bad",id) && !parseAprilTagId("",id) &&
        !parseAprilTagId(nullptr,id),8923);
  frame.count=255;
  for (auto &box:frame.boxes) {box=frame.boxes[4];strcpy(box.content,"255");}
  packAprilTags(frame,packet);
  check(packet.vision.amountoObjects==10 && packet.vision.objList[9].ID==255,8924);
  frame.count=0; packAprilTags(frame,packet);
  check(packet.vision.amountoObjects==0,8925);
}

static void heldTagPositionsAreNotIndependentSamples() {
  const calibH c=identityCalibration();
  BeltCalibrator b;
  // Camera updates every 280ms while GET_RESULT is polled every 20ms.
  // Sampling these held results every 80ms previously produced ~10mm RMS
  // for an exactly constant -125mm/s belt, like the supplied serial logs.
  for (unsigned ms=0;ms<=2100;ms+=20) {
    objectVision d=calibrationDetection(324,420-35*(ms/280));
    d.width=20+(ms/20)%2; // Size changes do not make an identical centre fresh.
    d.ID=0;
    check(b.addFrame(&d,1,1000+ms,c),8960);
  }
  float v=0;
  check(b.finish(v) && fabsf(v+125)<.001f && b.samples()==8 &&
        b.report().spanMs==1960 && b.report().repeatedPositions==98 &&
        b.report().rmsMm<.001f && b.lastSampleMs()==2960,8961);
  const uint32_t last=b.lastSampleMs();
  objectVision held=calibrationDetection(324,175); held.ID=0;
  check(b.addFrame(&held,1,4000,c) && b.lastSampleMs()==last && b.samples()==8,8962);
  b.reset();
  check(!b.seen() && b.report().repeatedPositions==0,8963);
  objectVision d=calibrationDetection(324,400);
  b.addFrame(&d,1,1000,c);
  d.yCenter=399; b.addFrame(&d,1,1020,c); // Changed but too close to the accepted sample.
  b.addFrame(&d,1,1100,c); // Must not relabel that old position with a new time.
  check(b.samples()==1 && b.lastSampleMs()==1000 && b.report().repeatedPositions==1,8964);
  d.yCenter=380; b.addFrame(&d,1,1200,c);
  check(b.samples()==2 && b.lastSampleMs()==1200,8965);
  b.reset();
  d=calibrationDetection(324,400);
  for (unsigned i=0;i<10;++i) b.addFrame(&d,1,1000+200*i,c);
  check(!b.finish(v) && b.samples()==1 && b.report().spanMs==0 &&
        b.report().repeatedPositions==9,8966); // No fictitious stationary fit.
  b.reset();
  for (unsigned ms=0;ms<=2100;ms+=25) {
    d=calibrationDetection(324,420-35*(ms/280));
    b.addFrame(&d,1,uint32_t(0xffffff00U+ms),c);
  }
  check(b.finish(v) && b.samples()==8 && fabsf(v+125)<2 && b.report().rmsMm<2,8967);
}

static void beginTagCalibrationHarness() {
  setupAutoHarness();
  // Bare-metal emulator enters test_main directly, without Arduino's global
  // constructor startup. Construct this global fixture explicitly.
  beltCalibration=BeltCalibrator{}; calPhase=CalPhase::Ending;
  camera_calib=identityCalibration();
  currentState=VELOCITY_CALIBRATION; lastState=CALIBRATION_MENU;
  pendingESPmsg.pending=false; fakeAckType=0;
  serviceVelocityCalibration(adapterTime,0);
  fakeAckType=CMD_START_STREAM;
  serviceVelocityCalibration(adapterTime,0);
}
static void tagClearFrame(uint32_t elapsed=250) {
  adapterTime+=elapsed;
  fillCornerTags(latestReceived); totalObjects=4;
  visionTimestampMs=adapterTime; newLatest=true;
  serviceVelocityCalibration(adapterTime,0);
}
static void clearTagLane() {
  for (unsigned i=0;i<=BeltCalibrationSessionConfig::laneClearMs/250;++i)
    tagClearFrame();
}
static void tagPass(unsigned step=40, unsigned samples=4) {
  for (unsigned i=0;i<samples;++i) {
    fillCornerTags(latestReceived);
    latestReceived[4]=calibrationDetection(324,420-step*i); latestReceived[4].ID=2;
    totalObjects=5; visionTimestampMs=adapterTime+400; adapterTime+=400; newLatest=true;
    serviceVelocityCalibration(adapterTime,0);
  }
  adapterTime+=1501;
  serviceVelocityCalibration(adapterTime,0);
}
static void controllerLossStopsOnlyMotionModes() {
  const State exempt[] = {CAMERA_CALIBRATION, VELOCITY_CALIBRATION, MODE_SELECTOR,
                         AUTO_MENU, CALIBRATION_MENU, TUNING_MENU, WAITING_ALGO_SWITCH, AUTO_MODE};
  for (State state : exempt) {
    for (unsigned flags=0;flags<3;++flags) {
      setupAutoHarness(); currentState=state; menu_option2=1;
      udpControllerStable=flags==2; controllerTimedOutThisLoop=flags!=0;
      const uint32_t writes=Serial1.writes;
      serviceImmediateControllerFaults(); serviceGlobalFaults();
      check(currentState==state && !entered_ES && Serial1.writes==writes,8990);
      // A received emergency edge still wins, even with the timeout flag set.
      controllerEmergencyReceived=true;
      serviceImmediateControllerFaults();
      check(currentState==EMERGENCY_STOP && entered_ES &&
            strcmp(Serial1.batch,"S:ESTOP\n")==0,8991);
      check(controllerLossRequiresStop(),9001); // Recovery cannot clear an ESTOP.
    }
    setupAutoHarness(); currentState=state; menu_option2=1; udpControllerStable=false;
    currentControlCMD.buttons=BTN_B; serviceGlobalFaults();
    check(currentState==EMERGENCY_STOP,8992);
    if (usesCamera(state)) {
      setupAutoHarness(); currentState=state; menu_option2=1; udpControllerStable=false;
      cameraOffline=true; serviceGlobalFaults();
      check(currentState==EMERGENCY_STOP,8993);
      setupAutoHarness(); currentState=state; menu_option2=1; udpControllerStable=false;
      communicationError=true; serviceGlobalFaults();
      check(currentState==EMERGENCY_STOP,8994);
    }
  }
  const State supervised[] = {MANUAL_0, MANUAL_1,
      AUTO_MODE, TRAJECTORY_TEST,
      JOINT_CALIBRATION, JOINT_MOTION_TEST};
  for (State state : supervised) {
    setupAutoHarness(); currentState=state; controllerTimedOutThisLoop=true;
    serviceImmediateControllerFaults();
    check(currentState==EMERGENCY_STOP && entered_ES,8995);
    setupAutoHarness(); currentState=state; udpControllerStable=false;
    serviceGlobalFaults();
    check(currentState==EMERGENCY_STOP && entered_ES,8996);
  }
  // Menus remain usable, but entering a motion mode restores supervision.
  setupAutoHarness(); currentState=VELOCITY_CALIBRATION; udpControllerStable=false;
  serviceGlobalFaults(); check(currentState==VELOCITY_CALIBRATION,8997);
  currentState=CALIBRATION_MENU; serviceGlobalFaults();
  check(currentState==CALIBRATION_MENU,9000);
  currentState=MANUAL_0; serviceGlobalFaults();
  check(currentState==EMERGENCY_STOP,8998);
}
static void actualTagCalibrationRequiresIndependentPasses() {
  beginTagCalibrationHarness();
  check(calPhase==CalPhase::Collecting && !velocityCalibrated,8940);
  // An occupied lane cannot start/reuse a pass before the operator prompt.
  latestReceived[0]=calibrationDetection(324,400); totalObjects=1;
  newLatest=true; visionTimestampMs=adapterTime;
  serviceVelocityCalibration(adapterTime,0);
  check(!beltCalibration.seen() && beltCalibrationSession.accepted==0,8941);
  clearTagLane();
  check(beltCalibrationSession.phase==BeltCalibrationSession::Phase::Ready,8942);
  Serial.clear(); tagPass(40,6);
  check(beltCalibrationSession.accepted==1 && !velocityCalibrated &&
        !mainTracker.calibrated() && calPhase==CalPhase::Collecting &&
        strstr(Serial.text,"Pass ACCEPTED"),8943);
  // No frame, a held in-lane result, or a partial box cannot rearm the pass.
  for (unsigned i=0;i<20;++i) { adapterTime+=100; serviceVelocityCalibration(adapterTime,0); }
  check(beltCalibrationSession.accepted==1,8944);
  for (unsigned i=0;i<12;++i) {
    adapterTime+=250; newLatest=true; visionTimestampMs=adapterTime;
    serviceVelocityCalibration(adapterTime,0);
  }
  check(beltCalibrationSession.phase==BeltCalibrationSession::Phase::Clearing &&
        beltCalibrationSession.accepted==1,8945);
  latestReceived[0]=calibrationDetection(224,400); totalObjects=1;
  check(velocityLaneOccupied(),8946); // Centre on ROI boundary; box crosses it.
  // Fresh empty frames are required: elapsed time with no camera data is insufficient.
  tagClearFrame(); adapterTime+=5000; tagClearFrame(0);
  check(beltCalibrationSession.phase==BeltCalibrationSession::Phase::Clearing,8947);
  clearTagLane();
  Serial.clear(); tagPass(40,2);
  check(beltCalibrationSession.accepted==1 && beltCalibrationSession.rejected==1 &&
        !velocityCalibrated && strstr(Serial.text,"TOO_FEW_SAMPLES"),8948);
  // Retry the failed run, then reach ten valid independent passes. Each gets
  // equal weight, despite a different number of positions in the first pass.
  for (unsigned run=1;run<BeltCalibrationSessionConfig::requiredRuns;++run) {
    Serial.clear(); clearTagLane();
    check(strstr(Serial.text,"PLACE TAG") &&
          beltCalibrationSession.phase==BeltCalibrationSession::Phase::Ready,8949);
    // Keep calibrating through a real link-down period and late same-loop
    // recovery. Neither supervisor may discard accepted passes or the stream.
    udpControllerStable=run!=3;
    controllerTimedOutThisLoop=(run==3 || run==4);
    serviceImmediateControllerFaults(); serviceGlobalFaults();
    check(currentState==VELOCITY_CALIBRATION && !entered_ES,8999);
    tagPass(40+run);
    check(beltCalibrationSession.accepted==run+1,8950);
    check(velocityCalibrated==(run+1==BeltCalibrationSessionConfig::requiredRuns),8951);
  }
  check(velocityCalibrated && fabsf(mainTracker.velocity()+111.25f)<.001f &&
        calPhase==CalPhase::Ending && lastCriticalType==CMD_END_STREAM,8952);
  check(fabsf(beltCalibrationSession.minimum+122.5f)<.001f &&
        fabsf(beltCalibrationSession.maximum+100)<.001f &&
        fabsf(beltCalibrationSession.standardDeviation()-7.569126f)<.001f &&
        strstr(Serial.text,"COMPLETE runs=10 rejected=1"),8953);
  fakeAckType=CMD_END_STREAM; serviceVelocityCalibration(adapterTime,0);
  check(currentState==CALIBRATION_MENU,8954);
}
static void actualTagCalibrationWaitsRetriesAndCancels() {
  beginTagCalibrationHarness();
  clearTagLane();
  Serial.clear();
  for (unsigned i=0;i<20;++i) tagClearFrame();
  check(!strstr(Serial.text,"[CAL SAMPLE]"),8981); // Quiet operator wait.
  adapterTime+=60000; serviceVelocityCalibration(adapterTime,0);
  check(!velocityCalibrated && !beltCalibration.seen() && calPhase==CalPhase::Collecting,8970);
  serviceVelocityCalibration(adapterTime,BTN_L1);
  check(calPhase==CalPhase::Ending && !velocityCalibrated &&
        lastCriticalType==CMD_END_STREAM,8971);
  fakeAckType=CMD_END_STREAM; serviceVelocityCalibration(adapterTime,0);
  check(currentState==CALIBRATION_MENU,8972);
  beginTagCalibrationHarness();
  clearTagLane();
  latestReceived[0]=calibrationDetection(300,400);
  latestReceived[1]=calibrationDetection(350,400);
  totalObjects=2; newLatest=true; visionTimestampMs=adapterTime;
  serviceVelocityCalibration(adapterTime,0);
  check(!velocityCalibrated && calPhase==CalPhase::Collecting &&
        beltCalibrationSession.rejected==1 && beltCalibrationSession.accepted==0 &&
        beltCalibration.report().issue==BeltCalibrationIssue::MultipleTags,8973);
  clearTagLane(); tagPass();
  check(beltCalibrationSession.accepted==1,8974);
  serviceVelocityCalibration(adapterTime,BTN_L1);
  check(!velocityCalibrated && !mainTracker.calibrated() && calPhase==CalPhase::Ending,8975);
  // Starting cancellation must wait for START ACK before sending END.
  currentState=VELOCITY_CALIBRATION; lastState=CALIBRATION_MENU;
  fakeAckType=0; serviceVelocityCalibration(adapterTime,0);
  serviceVelocityCalibration(adapterTime,BTN_L1);
  check(calPhase==CalPhase::Starting && lastCriticalType==CMD_START_STREAM,8976);
  fakeAckType=CMD_START_STREAM; serviceVelocityCalibration(adapterTime,0);
  check(calPhase==CalPhase::Ending && lastCriticalType==CMD_END_STREAM && !velocityCalibrated,8977);
  // Clearing uses wrap-safe durations and does not accept a long missing-data gap.
  BeltCalibrationSession s;
  check(!s.observeClear(false,0xffffff00U),8978);
  for (unsigned ms=250;ms<=1500;ms+=250) s.observeClear(false,uint32_t(0xffffff00U+ms));
  check(s.phase==BeltCalibrationSession::Phase::Ready,8979);
  check(!s.record(NAN) && !s.record(100) && s.accepted==0,8980);
}


static void scheduledPickupAndConcurrentClosing() {
  for (unsigned tall=0; tall<2; ++tall)
    for (uint8_t bin=1;bin<=3;++bin) {
    Fixture f;
    f.config=TrajectoryTest::configuration(f.config,332,f.params,bin);
    f.config.closingLeadMs=300;
    check(f.config.pickUpBudgetMs()==1600 && f.pick(),9200);
    check(f.until("WAIT FOR APPROACH") && positionCommands==0,9201);
    check(f.until("APPROACH") && uint32_t(f.now-f.startTime)==2400,9202);
    float abovePose[6]; robot_fkine(commanded,f.params,abovePose);
    check(fabsf(abovePose[3])<1 && fabsf(abovePose[4]+PickMotion::surfaceTiltDegrees())<1 &&
          fabsf(abovePose[5])<1,9203);
    check(f.until("SAFETY HEIGHT WAIT"),9204);
    Part box; box.trackId=1; box.boxWidth=tall ? 20:40; box.boxHeight=tall ? 40:20;
    box.x=777; box.y=-100; box.classId=99; box.stateMs=f.now;
    f.arm.updateObservation(box); // Only metadata may update, never timing/position/route.
    check(f.until("DESCEND") && uint32_t(f.now-f.startTime)==3400 &&
          f.arm.orientationLocked() && f.arm.selectedPickupYaw()==(tall?90:0),9205);
    float pickup[6]; robot_fkine(commanded,f.params,pickup);
    check(hypotf(hypotf(pickup[0]-332,pickup[1]),pickup[2]-PickMotion::beltContactZ(332))<.5f,9206);
    check(fabsf(pickup[3]-(tall ? -PickMotion::surfaceTiltDegrees():0))<1 &&
          fabsf(pickup[4]-(tall ? 0:-PickMotion::surfaceTiltDegrees()))<1,9207);
    // Refuse a different track and a post-descent flip of the assigned track.
    box.boxWidth=tall ? 40:20; box.boxHeight=tall ? 20:40; f.arm.updateObservation(box);
    const float q5=commanded[4];
    while(uint32_t(f.now-f.startTime)<3700) f.tick(10,false);
    check(sawClose && closeTime-f.startTime==3700 && f.stage("CLOSE") &&
          !f.arm.progress(f.feedback,f.now).withinTarget,9208);
    // The claw starts before descent has settled, without causing pose-lost.
    check(f.until("CLASSIFICATION WAYPOINT") && f.now-f.startTime>=4220 &&
          fabsf(commanded[4]-q5)<.005f,9209);
    check(f.finish()==PickOutcome::Sorted && gripperCommands==2 && positionCommands==4,9210);
  }
}
static void missedPickupReturnsHomeWithoutRelease() {
  Fixture late;
  late.config.closingLeadMs=300;
  check(late.pick() && late.until("APPROACH"),9220);
  while(late.arm.arrivalDelayMs(late.now)>=late.config.descentLeadMs) late.tick(10,false);
  check(!strcmp(late.arm.missReason(),"SAFETY_HEIGHT_LATE") && !sawClose,9221);
  check(late.finish()==PickOutcome::Missed && !late.arm.clawMayHoldPart() && gripperCommands==0,9222);
  for(float q:commanded) check(q==0,9223);
  late.tick(10000); check(!sawClose && !late.arm.busy(),9224);

  Fixture held; held.config.closingLeadMs=300;
  check(held.pick() && held.until("DESCEND"),9225);
  while(held.arm.arrivalDelayMs(held.now)>0) held.tick(10,false);
  check(sawClose && !strcmp(held.arm.missReason(),"GRIPPING_HEIGHT_LATE") &&
        held.arm.clawMayHoldPart(),9226);
  check(held.finish()==PickOutcome::Missed && gripperCommands==1 && held.arm.clawMayHoldPart(),9227);
  for(float q:commanded) check(q==0,9228);
  held.tick(10000); check(gripperCommands==1 && !held.arm.busy(),9229);

  Fixture cancelled; check(cancelled.pick() && cancelled.until("CLOSE"),9230);
  cancelled.arm.cancelPickup(cancelled.feedback,cancelled.now);
  check(cancelled.finish()==PickOutcome::Missed && cancelled.arm.clawMayHoldPart() &&
        gripperCommands==1 && !strcmp(cancelled.arm.missReason(),"OPERATOR_CANCELLED"),9231);
}
static void scheduledBoundaryJitterAndClockWrap() {
  Fixture exact(0xfffff000U);
  exact.config.closingLeadMs=300;
  check(exact.pick() && exact.until("APPROACH"),9240);
  exact.tick(800,false); exact.tick(100,true); exact.tick(100,true);
  check(exact.stage("SAFETY HEIGHT WAIT") && exact.arm.arrivalDelayMs(exact.now)==600,9241);
  exact.tick(10); // First scheduler iteration lands 10 ms after deadline.
  check(exact.stage("DESCEND") && !*exact.arm.missReason(),9242);
  check(exact.finish()==PickOutcome::Sorted,9243);

  Fixture missed; check(missed.pick() && missed.until("SAFETY HEIGHT WAIT"),9244);
  const uint32_t delay=uint32_t(missed.arm.arrivalDelayMs(missed.now)-missed.config.descentLeadMs);
  missed.tick(delay+missed.config.scheduleToleranceMs+1);
  check(!strcmp(missed.arm.missReason(),"DESCENT_COMMAND_LATE") && !sawClose,9245);
  check(missed.finish()==PickOutcome::Missed && gripperCommands==0,9246);

  Fixture plannedLate; check(plannedLate.pick(),9247);
  plannedLate.tick(2500);
  check(!strcmp(plannedLate.arm.missReason(),"PLAN_LATE") && !sawClose,9248);
  check(plannedLate.finish()==PickOutcome::Missed,9249);
}
static void trackingRectangleAndReservedLastBox() {
  BeltTracker tracker(0); tracker.setVelocity(-100);
  calibH c=identityCalibration();
  objectVision d=calibrationDetection(332,440); d.width=20; d.height=20; d.ID=partForBin(1);
  tracker.updateFrame(&d,1,900,c);
  Part p; check(!tracker.candidate(0,p,900),9250);
  for(unsigned i=0;i<3;++i) {
    d.yCenter=uint16_t(400-i*10); d.height=40;
    tracker.updateFrame(&d,1,1000+i*100,c);
  }
  check(tracker.candidate(0,p,1200) && p.boxWidth==20 && p.boxHeight==40,9251);
  const uint32_t id=p.trackId;
  check(tracker.reserve(id),9252);
  d.yCenter=370; d.width=40; d.height=20; d.ID=99;
  tracker.updateFrame(&d,1,1300,c);
  check(tracker.observation(id,p) && p.classId==partForBin(1) && p.pendingClass==99 &&
        p.boxWidth==40 && p.boxHeight==20 && p.observedMs==1300,9253);
  d.yCenter=50; d.height=40; d.ID=8; // Clipped above ROI: must affect nothing.
  tracker.updateFrame(&d,1,1400,c);
  tracker.tick(1500);
  check(tracker.observation(id,p) && p.boxWidth==40 && p.boxHeight==20 &&
        p.observedMs==1300 && p.pendingClass==99 && fabsf(p.y-350)<.1f,9254);
  for(unsigned i=0;i<MAX_PARTS;++i) check(!tracker.candidate(i,p,1500),9255);
  const auto r=CameraTracking::PartRegion::rectangle;
  check(r.containsBox(74,58,20,20) && !r.containsBox(74,58,21,20) &&
        r.containsBox(566,422,20,20) && !r.containsBox(566,422,20,21),9256);
}

static void autoUsesReservedLastBoxAndStopsIfRecoveryHoldsPart() {
  setupAutoHarness(-100); check(startAutoHarness(),9260);
  mainTracker=BeltTracker(CameraTracking::partXOffsetMm); mainTracker.setVelocity(-100);
  camera_calib.H(1,2)=240; // Real 640x480 pixels, world Y=660 at pixel Y=420.
  for(unsigned i=0;i<7;++i) {
    latestReceived[0]=calibrationDetection(322,uint16_t(420-10*i));
    latestReceived[0].width=20; latestReceived[0].height=20;
    latestReceived[0].ID=partForBin(1); totalObjects=1;
    visionTimestampMs=adapterTime+100; newLatest=true; autoTick(100);
  }
  check(arm.busy() && arm.trackId()==1,9261);
  // Class/bin are frozen, but this later valid orientation must win.
  latestReceived[0].yCenter=350; latestReceived[0].width=20;
  latestReceived[0].height=40; latestReceived[0].ID=partForBin(3);
  visionTimestampMs=adapterTime+100; newLatest=true; autoTick(100);
  latestReceived[0].yCenter=50; latestReceived[0].width=40; latestReceived[0].height=40;
  visionTimestampMs=adapterTime+100; newLatest=true; autoTick(100); // Top edge clips ROI.
  Part reserved;
  check(mainTracker.observation(arm.trackId(),reserved) && reserved.boxWidth==20 &&
        reserved.boxHeight==40 && reserved.classId==partForBin(1),9262);
  check(autoUntil("DESCEND") && arm.selectedPickupYaw()==90,9263);
  check(autoUntil("CLASSIFICATION POSE"),9264);
  float pose[6]; robot_fkine(commanded,params,pose);
  const float *expected=autoConfig.destinations[0].pose;
  check(hypotf(hypotf(pose[0]-expected[0],pose[1]-expected[1]),pose[2]-expected[2])<=.5f,9265);

  setupAutoHarness(-100); check(startAutoHarness(),9270);
  const uint32_t openingCount=gripperCommands;
  // Cancelling a TALL pickup after closing must retain the held part at HOME.
  feedAutoTrack(332,660,partForBin(1),-100,0,true);
  check(autoUntil("DESCEND"),9271);
  while(!sawClose && arm.busy()) autoTick(20,0,false);
  arm.cancelPickup(armFeedback,adapterTime);
  check(sawClose && arm.clawMayHoldPart() &&
        !strcmp(arm.missReason(),"OPERATOR_CANCELLED"),9272);
  for(unsigned i=0;i<100 && autoRun!=AutoRun::Ending;++i) autoTick();
  check(autoRun==AutoRun::Ending && auto_parts_missed==1 && !arm.busy() &&
        arm.clawMayHoldPart() && gripperCommands==openingCount+1,9273);
  for(float q:commanded) check(q==0,9274);
  fakeAckType=CMD_END_STREAM; autoTick();
  check(autoRun==AutoRun::Stopped && !streamStarted && arm.clawMayHoldPart(),9275);
  autoTick(500, BTN_Y); check(gripperCommands==openingCount+1,9276);
  autoTick(20, BTN_A); fakeAckType=CMD_START_STREAM; autoTick();
  check(autoRun==AutoRun::Preparing && gripperCommands==openingCount+2 &&
        !arm.clawMayHoldPart(),9277); // Explicit A acknowledges release at HOME.
}

static bool diagnosticOf(ArmTrajectory &controller, ArmDiagnosticEvent event, ArmDiagnosticReport &out) {
  ArmDiagnosticReport d; bool found=false;
  while(controller.takeDiagnostic(d)) if(d.event==event) {out=d; found=true;}
  return found;
}
static void missDiagnosticsRetainFailureTargetAndActualSettings() {
  const uint32_t margins[]={1000,1100,1200};
  for(uint32_t margin : margins) {
    Fixture f; f.config.pickUpMarginMs=margin; f.config.settleMs=150;
    check(f.pick() && f.until("APPROACH"),9300);
    const uint32_t approachMs=f.now;
    float failedTarget[5]; memcpy(failedTarget,commanded,sizeof(failedTarget));
    // Poll every millisecond: the first failed check is 599 ms remaining.
    while(!*f.arm.missReason()) f.tick(1,false);
    ArmDiagnosticReport d;
    check(diagnosticOf(f.arm,ArmDiagnosticEvent::Missed,d),9301);
    check(d.remainingArrivalMs==599 && d.progress.elapsedMs==margin+1 &&
          d.progress.tickGapMs==1 && d.progress.feedbackIntervalMs==1 &&
          !strcmp(d.stage,"APPROACH") && !strcmp(d.targetName,"SAFETY_HEIGHT") &&
          d.marginMs==margin && d.descentLeadMs==600 && d.thresholdMs==600,9302);
    check(d.progress.elapsedMs==f.now-approachMs && !d.progress.withinTarget &&
          d.progress.stableMs==0 && d.progress.positionErrorMm>5 &&
          memcmp(d.progress.target,failedTarget,sizeof(failedTarget))==0,9303);
    Print log; printArmDiagnostic(log,d);
    check(strstr(log.text,"requirement=SAFETY_CONFIRMED_BEFORE_DESCENT") &&
          strstr(log.text,"thresholdRemaining=600.0ms margin=-1.0ms") &&
          strstr(log.text,"tickGap=1ms") && strstr(log.text,"[PICK JOINTS]"),9304);
    check(f.finish()==PickOutcome::Missed,9305);
    // HOME has replaced the live target, but the captured failure is unchanged.
    for(float q:commanded) check(q==0,9306);
    check(memcmp(d.progress.target,failedTarget,sizeof(failedTarget))==0 &&
          d.progress.positionErrorMm>5,9307);
  }
}
static void grippingMissSeparatesSettlingFromDescentDeadline() {
  Fixture f; f.config.closingLeadMs=300; f.config.settleMs=150;
  check(f.pick() && f.until("DESCEND"),9320);
  float gripTarget[5]; memcpy(gripTarget,commanded,sizeof(gripTarget));
  // The arm reaches its pose, but only 50 ms of stable evidence precedes Y=0.
  while(f.arm.arrivalDelayMs(f.now)>50) f.tick(10,false);
  f.tick(0,true); f.tick(50,true);
  check(!strcmp(f.arm.missReason(),"GRIPPING_HEIGHT_LATE") && f.arm.clawMayHoldPart(),9321);
  ArmDiagnosticReport d;
  check(diagnosticOf(f.arm,ArmDiagnosticEvent::Missed,d),9322);
  check(!strcmp(d.stage,"CLOSE") && !strcmp(d.targetName,"GRIPPING_HEIGHT") &&
        !strcmp(d.requirement,"GRIPPING_CONFIRMED_BY_Y0") && d.thresholdMs==0 &&
        d.remainingArrivalMs==0 && d.progress.withinTarget && d.progress.stableMs==50 &&
        d.settleMs==150 && d.progress.elapsedMs==600 && d.closeElapsedMs==300 &&
        memcmp(d.progress.target,gripTarget,sizeof(gripTarget))==0,9323);
  Print log; printArmDiagnostic(log,d);
  check(strstr(log.text,"thresholdRemaining=0.0ms") &&
        !strstr(log.text,"requiredDescentLead") && strstr(log.text,"jointsOK=1 TCPok=1") &&
        strstr(log.text,"stable=50/150ms") && strstr(log.text,"closeStarted=1 closeElapsed=300ms"),9324);
  check(f.finish()==PickOutcome::Missed && gripperCommands==1,9325);
  log.clear(); printArmDiagnostic(log,d);
  check(strstr(log.text,"target=GRIPPING_HEIGHT") && strstr(log.text,"claw=CLOSED"),9326);
}
static void confirmationDiagnosticsShowHeadroomAndDoNotConsumeEventsTwice() {
  Fixture f; f.config.closingLeadMs=300;
  check(f.pick() && f.until("SAFETY HEIGHT WAIT"),9340);
  ArmDiagnosticReport safety;
  check(diagnosticOf(f.arm,ArmDiagnosticEvent::SafetyConfirmed,safety),9341);
  check(safety.remainingArrivalMs>600 && safety.progress.stableMs>=safety.settleMs &&
        safety.progress.elapsedMs>=safety.minimumMoveMs && safety.marginMs==1000,9342);
  Print log; printArmDiagnostic(log,safety);
  check(strstr(log.text,"[PICK TIMING] SAFETY_CONFIRMED") &&
        strstr(log.text,"deadlineMargin=+") && !strstr(log.text,"[PICK MISSED]"),9343);
  ArmDiagnosticReport none;
  check(!f.arm.takeDiagnostic(none),9344);
  check(f.until("DESCEND"),9345);
  ArmDiagnosticReport descent;
  check(diagnosticOf(f.arm,ArmDiagnosticEvent::DescentCommand,descent) &&
        descent.remainingArrivalMs==600 && descent.progress.elapsedMs==0,9346);
  check(f.until("WAIT FOR PART"),9347);
  ArmDiagnosticReport grip;
  check(diagnosticOf(f.arm,ArmDiagnosticEvent::GrippingConfirmed,grip) &&
        grip.remainingArrivalMs>0 && grip.progress.elapsedMs>=grip.minimumMoveMs,9348);
  check(f.until("CLOSE"),9349);
  ArmDiagnosticReport close;
  check(diagnosticOf(f.arm,ArmDiagnosticEvent::CloseCommand,close) &&
        close.remainingArrivalMs==300 && close.closingStarted && close.closeElapsedMs==0,9350);
  check(f.finish()==PickOutcome::Sorted && positionCommands==4 && gripperCommands==2,9351);
}

static SavedRobotSettings exampleSavedSettings() {
  SavedRobotSettings s;s.tuning=RuntimeTuning::defaults();
  s.tuning.partXOffsetMm=15.5f;
  s.homography=identityCalibration();s.velocityValid=true;s.velocity=-126.5f;return s;
}
static void settingsCodecAndJournalSurviveInterruptedWrites() {
  MemorySettingsFlash io;io.reset();SettingsJournal<MemorySettingsFlash> store(io);
  SavedRobotSettings s=exampleSavedSettings(),out;
  check(!store.load(out) && store.available && store.generation==0 && io.erasures==0,9500);
  check(store.save(s) && store.generation==1,9501);
  uint8_t first[8192];memcpy(first,io.bytes,sizeof(first));
  s.tuning.descentMs+=10;
  for(unsigned cut=0;cut<=SettingsRecord::size;++cut) {
    memcpy(io.bytes,first,sizeof(first));io.operation=0;io.failAt=0;
    SettingsJournal<MemorySettingsFlash> writer(io);check(writer.load(out),9502);
    io.failAt=2;io.partialBytes=int(cut);
    check(!writer.save(s),9503);
    io.failAt=0;SettingsJournal<MemorySettingsFlash> reboot(io);
    check(reboot.load(out) && reboot.generation==1 && out.tuning.descentMs==s.tuning.descentMs-10,9504);
    check(memcmp(first,io.bytes,io.bankSize)==0,9505);
  }
  for(unsigned cut=0;cut<=4;++cut) {
    memcpy(io.bytes,first,sizeof(first));io.operation=0;io.failAt=0;
    SettingsJournal<MemorySettingsFlash> writer(io);check(writer.load(out),9506);
    io.failAt=3;io.partialBytes=cut;check(!writer.save(s),9507);
    io.failAt=0;SettingsJournal<MemorySettingsFlash> reboot(io);
    check(reboot.load(out) && reboot.generation==(cut==4?2U:1U),9508);
  }
  memcpy(io.bytes,first,sizeof(first));io.operation=0;io.failAt=0;
  SettingsJournal<MemorySettingsFlash> writer(io);check(writer.load(out) && writer.save(s),9509);
  io.bytes[4096+20]^=1;SettingsJournal<MemorySettingsFlash> corrupt(io);
  check(corrupt.load(out) && corrupt.generation==1,9510);
  io.reset();io.bytes[500]=0;SettingsJournal<MemorySettingsFlash> occupied(io);
  check(!occupied.load(out) && !occupied.available && !occupied.save(s) && io.erasures==0,9511);
  uint8_t record[SettingsRecord::size];SettingsRecord::encode(s,8,record);
  check(SettingsRecord::decode(record,out,store.generation) && out.homography.valid &&
        out.velocityValid && out.velocity==s.velocity && out.tuning.partXOffsetMm==15.5f,9512);
  record[4]^=1;check(!SettingsRecord::decode(record,out,store.generation),9513);
  s.homography.H(0,0)=NAN;check(!SettingsRecord::valid(s),9514);
  s=exampleSavedSettings();s.velocity=100;check(!SettingsRecord::valid(s),9515);
  s.velocityValid=false;s.homography.valid=false;check(SettingsRecord::valid(s),9516);
}
static void legacySettingsRetainTuningAndCalibrationsWhenOffsetIsAdded() {
  // Frozen schema-1 fixture, with an independently computed CRC32. It stores
  // margin=2500/descent=760/closeLead=420, identity H and velocity=-126.5.
  static const uint8_t legacy[128]={
    0x52,0x42,0x54,0x33,0x01,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x64,0x00,0x00,0x00,
    0xc4,0x09,0x00,0x00,0xf8,0x02,0x00,0x00,0xa4,0x01,0x00,0x00,0x08,0x02,0x00,0x00,
    0xf4,0x01,0x00,0x00,0x00,0x00,0x20,0x42,0x00,0x00,0xa0,0xc0,0x7f,0x6a,0x20,0xc2,
    0x00,0x00,0x96,0x43,0x00,0x00,0xa6,0x43,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x80,0x3f,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x3f,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x3f,0x01,0x00,0x00,0x00,
    0x00,0x00,0xfd,0xc2,0x85,0xe7,0x0c,0x15,0xff,0xff,0xff,0xff,0x3d,0xc0,0xa6,0x51,
  };
  MemorySettingsFlash io;io.reset();memcpy(io.bytes,legacy,sizeof(legacy));
  SettingsJournal<MemorySettingsFlash> store(io);SavedRobotSettings s,out;
  check(store.load(s) && store.generation==3 && io.erasures==0 &&
        s.tuning.marginMs==2500 && s.tuning.descentMs==760 && s.tuning.closingLeadMs==420 &&
        s.tuning.approachZ==40 && s.tuning.heightAdjustment==-5 && s.homography.valid &&
        s.homography.H(0,0)==1 && s.velocityValid && s.velocity==-126.5f &&
        s.tuning.partXOffsetMm==CameraTracking::partXOffsetMm,9700);
  s.tuning.partXOffsetMm=12.5f;
  check(store.save(s) && store.generation==4 && memcmp(io.bytes,legacy,sizeof(legacy))==0 &&
        SettingsRecord::get32(io.bytes+4096+4)==SettingsRecord::schema,9701);
  SettingsJournal<MemorySettingsFlash> reboot(io);
  check(reboot.load(out) && reboot.generation==4 && out.tuning.partXOffsetMm==12.5f &&
        out.tuning.marginMs==2500 && out.tuning.heightAdjustment==-5 &&
        out.homography.valid && out.velocityValid && out.velocity==-126.5f,9702);
  io.bytes[4096+116]^=1; // New offset is inside the version-2 CRC coverage.
  SettingsJournal<MemorySettingsFlash> corrupt(io);
  check(corrupt.load(out) && corrupt.generation==3 &&
        out.tuning.partXOffsetMm==CameraTracking::partXOffsetMm && out.velocity==-126.5f,9703);
  for(unsigned cut=0;cut<=SettingsRecord::size;++cut) {
    io.reset();memcpy(io.bytes,legacy,sizeof(legacy));
    SettingsJournal<MemorySettingsFlash> writer(io);check(writer.load(out),9704);
    io.failAt=2;io.partialBytes=cut;check(!writer.save(s),9705);
    io.failAt=0;SettingsJournal<MemorySettingsFlash> interrupted(io);
    check(interrupted.load(out) && interrupted.generation==3 && out.tuning.marginMs==2500 &&
          out.velocityValid && out.velocity==-126.5f && out.homography.valid &&
          memcmp(io.bytes,legacy,sizeof(legacy))==0,9706);
  }
}
static void storageLayoutNeverOverlapsExistingPartitions() {
  uint8_t mbr[512]{};mbr[510]=0x55;mbr[511]=0xaa;
  uint8_t *entry=mbr+446;entry[4]=0x0b;SettingsRecord::put32(entry+8,1);
  SettingsRecord::put32(entry+12,14U*1024*1024/512-1);
  check(!PortentaStorageLayout::issue(mbr,16U*1024*1024,4096),9520);
  SettingsRecord::put32(entry+12,14U*1024*1024/512+1);
  check(PortentaStorageLayout::issue(mbr,16U*1024*1024,4096)!=nullptr,9521);
  mbr[510]=0;check(PortentaStorageLayout::issue(mbr,16U*1024*1024,4096)!=nullptr,9522);
  mbr[510]=0x55;check(PortentaStorageLayout::issue(mbr,8U*1024*1024,4096)!=nullptr,9523);
}
static void runtimeTuningUsesExactStepsAndSharedGeometry() {
  RuntimeTuning t=RuntimeTuning::defaults();AutoConfig c;
  const float slope=PickMotion::heightSlope,open=c.openAngle,close=c.closeAngle;
  const uint32_t margin=t.marginMs;
  PickupTuningMenu::adjust(t,PickupTuningMenu::Margin,1);check(t.marginMs==margin+10,9530);
  PickupTuningMenu::adjust(t,PickupTuningMenu::Margin,-1);check(t.marginMs==margin,9531);
  const float contact=t.contactZ,reference=t.referenceX;
  PickupTuningMenu::adjust(t,PickupTuningMenu::ContactZ,1);TuningMenu::adjust(t,TuningMenu::ReferenceX,-1);
  check(t.contactZ==contact+.5f && t.referenceX==reference-.5f,9532);
  t.heightAdjustment=-2;t.apply(c);
  AutoPick::Request request;Part p;p.x=332;p.y=660;p.stateMs=100;
  check(AutoPick::evaluate(p,-100,c,false,100,request)==AutoPick::Admission::Eligible,9533);
  AutoConfig test=TrajectoryTest::configuration(c,332,robot_get_dh_params(),1);
  const float expected=t.contactZ+t.heightAdjustment+slope*(332-t.referenceX);
  check(fabsf(request.config.pickupZ-expected)<.001f && fabsf(test.pickupZ-expected)<.001f &&
        c.openAngle==open && c.closeAngle==close && PickMotion::heightSlope==slope && !t.issue(),9534);
  t.descentMs=t.closingLeadMs-10;check(t.issue()!=nullptr,9535);
  t=RuntimeTuning::defaults();t.marginMs+=1;check(t.issue()!=nullptr,9536);
  t=RuntimeTuning::defaults();t.referenceX=NAN;check(t.issue()!=nullptr,9537);
  t=RuntimeTuning::defaults();const float offset=t.partXOffsetMm;
  PickupTuningMenu::adjust(t,PickupTuningMenu::PartXOffset,1);check(t.partXOffsetMm==offset+.5f,9710);
  PickupTuningMenu::adjust(t,PickupTuningMenu::PartXOffset,-1);check(t.partXOffsetMm==offset,9711);
  t.partXOffsetMm=100;PickupTuningMenu::adjust(t,PickupTuningMenu::PartXOffset,1);check(t.partXOffsetMm==100,9712);
  t.partXOffsetMm=-100;PickupTuningMenu::adjust(t,PickupTuningMenu::PartXOffset,-1);check(t.partXOffsetMm==-100,9713);
  t.partXOffsetMm=100.5f;check(t.issue()!=nullptr,9714);
  t.partXOffsetMm=NAN;check(t.issue()!=nullptr,9715);
}
static void controllerMenuAppliesSavesReloadsAndKeepsCalibrationsIndependent() {
  setupAutoHarness(-126.5f);settingsFlash.reset();
  new (&settingsStore) SettingsJournal<MemorySettingsFlash>(settingsFlash);
  currentState=TUNING_MENU;lastState=CALIBRATION_MENU;tuningReturnState=TRAJECTORY_TEST;
  currentControlCMD.jsLx=currentControlCMD.jsLy=0;Serial.clear();
  const uint32_t commands=positionCommands,grips=gripperCommands;
  serviceTuningMenu(adapterTime,0);
  check(currentState==TUNING_MENU && !controllerLossRequiresStop() &&
        !strcmp(Serial1.batch,"\nS:MODES\n"),9550);
  const float old=activeTuning.referenceX;
  currentControlCMD.jsLx=1;serviceTuningMenu(++adapterTime,0);
  check(tuningDraft.referenceX==old+.5f && activeTuning.referenceX==old,9551);
  currentControlCMD.jsLx=0;serviceTuningMenu(++adapterTime,BTN_A);
  check(activeTuning.referenceX==old+.5f && autoConfig.heightReferenceXmm==old+.5f,9552);
  tuningDraft.partXOffsetMm+=.5f;applyTuningDraft();
  check(activeTuning.partXOffsetMm==CameraTracking::partXOffsetMm+.5f &&
        mainTracker.cameraXOffsetMm()==activeTuning.partXOffsetMm,9716);
  tuningRow=TuningMenu::Save;Serial.clear();serviceTuningMenu(++adapterTime,BTN_X);
  check(settingsStore.generation==1 && strstr(Serial.text,"SAVE ALL verified") &&
        positionCommands==commands && gripperCommands==grips,9553);
  const float savedSpeed=mainTracker.velocity();const uint32_t savedMargin=activeTuning.marginMs;
  camera_calib.valid=false;homographyDone=false;velocityCalibrated=mainTracker.setVelocity(0);
  tuningDraft.marginMs+=10;tuningDraft.partXOffsetMm+=10;
  applyTuningDraft();tuningRow=TuningMenu::Reload;
  serviceTuningMenu(++adapterTime,BTN_X);
  check(homographyDone && camera_calib.valid && velocityCalibrated && mainTracker.velocity()==savedSpeed &&
        activeTuning.marginMs==savedMargin &&
        mainTracker.cameraXOffsetMm()==CameraTracking::partXOffsetMm+.5f,9554);
  serviceTuningMenu(++adapterTime,BTN_L1);
  check(currentState==TRAJECTORY_TEST && positionCommands==commands && gripperCommands==grips,9555);

  currentState=CAMERA_CALIBRATION;lastState=CALIBRATION_MENU;
  serviceHomographyCalibration(++adapterTime,0);
  check(!homographyDone && velocityCalibrated && mainTracker.calibrated() &&
        mainTracker.velocity()==savedSpeed,9560);
  fakeAckType=CMD_START_STREAM;serviceHomographyCalibration(++adapterTime,0);
  const float wx[]={TAG0_WORLD_X,TAG1_WORLD_X,TAG2_WORLD_X,TAG3_WORLD_X};
  const float wy[]={TAG0_WORLD_Y,TAG1_WORLD_Y,TAG2_WORLD_Y,TAG3_WORLD_Y};
  for(unsigned i=0;i<4;++i) {
    latestReceived[i]=calibrationDetection(uint16_t(wx[i]/2+20),uint16_t(wy[i]/2-100));
    latestReceived[i].ID=i;
  }
  totalObjects=4;newLatest=true;serviceHomographyCalibration(++adapterTime,0);
  check(homographyDone && camera_calib.valid && velocityCalibrated && mainTracker.velocity()==savedSpeed &&
        calPhase==CalPhase::Ending,9561);
  fakeAckType=CMD_END_STREAM;serviceHomographyCalibration(++adapterTime,0);
  check(currentState==CALIBRATION_MENU && settingsStore.generation==1,9562); // Explicit SAVE required.
  const float savedH00=camera_calib.H(0,0);
  currentState=TUNING_MENU;lastState=CALIBRATION_MENU;serviceTuningMenu(++adapterTime,0);
  tuningRow=TuningMenu::Save;serviceTuningMenu(++adapterTime,BTN_X);
  if(settingsStore.generation!=2) snprintf(runtime_settings_debug,sizeof(runtime_settings_debug),
      "gen=%lu error=%s pending=%u stable=%u state=%u H=%u validSaved=%u tuning=%s serial=%s",
      (unsigned long)settingsStore.generation,settingsStore.error?settingsStore.error:"none",
      pendingESPmsg.pending,udpControllerStable,unsigned(currentState),homographyDone,
      SettingsRecord::valid(snapshotRuntimeSettings()),tuningDraft.issue()?tuningDraft.issue():"OK",Serial.text);
  check(settingsStore.generation==2,9564);
  homographyDone=camera_calib.valid=false;velocityCalibrated=mainTracker.setVelocity(0);
  loadRuntimeSettings();
  check(homographyDone && velocityCalibrated && mainTracker.velocity()==savedSpeed &&
        camera_calib.H(0,0)==savedH00,9565);
  currentState=CAMERA_CALIBRATION;lastState=CALIBRATION_MENU;
  serviceHomographyCalibration(++adapterTime,0);fakeAckType=CMD_START_STREAM;
  serviceHomographyCalibration(++adapterTime,BTN_L1);
  check(!homographyDone && velocityCalibrated && calPhase==CalPhase::Ending,9563);
}
static void runtimeCameraOffsetChangesTrackingAdmissionAndVisionDiagnosticsTogether() {
  setupAutoHarness(-100,true,false);
  tuningDraft.partXOffsetMm=15.5f;check(applyTuningDraft() && startAutoHarness(),9720);
  for(int i=0;i<3;++i) {
    latestReceived[0]=calibrationDetection(300,400-10*i);totalObjects=1;
    newLatest=true;visionTimestampMs=adapterTime+100;autoTick(100);
  }
  Serial.clear();autoTick(300);
  Part p;check(mainTracker.candidate(0,p,adapterTime) && p.x==315.5f &&
               strstr(Serial.text,"world={315.50,380.00}mm"),9721);
  AutoPick::Request request;
  check(AutoPick::evaluate(p,-100,autoConfig,false,adapterTime,request)==AutoPick::Admission::Eligible &&
        request.part.x==315.5f &&
        fabsf(request.config.pickupZ-PickMotion::beltContactZ(315.5f,autoConfig))<.001f,9722);
  const uint32_t oldTrack=p.trackId;
  tuningDraft.marginMs+=10;check(applyTuningDraft() && mainTracker.candidate(0,p,adapterTime) &&
        p.trackId==oldTrack,9723); // Unrelated tuning does not clear tracks.
  tuningDraft.partXOffsetMm=101;check(!applyTuningDraft() &&
        activeTuning.partXOffsetMm==15.5f && mainTracker.cameraXOffsetMm()==15.5f &&
        mainTracker.candidate(0,p,adapterTime),9724);
  tuningDraft.partXOffsetMm=-4.5f;newLatest=true;
  check(applyTuningDraft() && !newLatest && !mainTracker.candidate(0,p,adapterTime) &&
        mainTracker.calibrated() && mainTracker.velocity()==-100 && homographyDone && camera_calib.valid,9725);
  for(int i=0;i<3;++i) {
    latestReceived[0]=calibrationDetection(300,400-10*i);newLatest=true;
    visionTimestampMs=adapterTime+100;autoTick(100);
  }
  check(mainTracker.candidate(0,p,adapterTime) && p.x==295.5f && p.trackId!=oldTrack,9726);
  check(!mainTracker.setCameraXOffsetMm(NAN) && mainTracker.cameraXOffsetMm()==-4.5f &&
        mainTracker.candidate(0,p,adapterTime),9727);
  mainTracker.setVelocity(-120);
  check(mainTracker.cameraXOffsetMm()==-4.5f,9728); // Speed changes do not reset the offset.
  setupAutoHarness(-100);check(arm.prepare(autoConfig,armFeedback,adapterTime),9729);
  tuningDraft.partXOffsetMm=25;
  check(!applyTuningDraft() && activeTuning.partXOffsetMm==CameraTracking::partXOffsetMm &&
        mainTracker.cameraXOffsetMm()==CameraTracking::partXOffsetMm,9730);
}
static void automaticTuningRequestFinishesCurrentCycleAndStops() {
  setupAutoHarness(-100);check(startAutoHarness(),9570);
  feedAutoTrack(332,660,partForBin(1),-100);check(autoUntil("DESCEND"),9571);
  autoTick(20,BTN_R3);check(tuningMenuRequested && stopRequested && arm.busy(),9572);
  for(unsigned i=0;i<700 && autoRun!=AutoRun::Ending && currentState==AUTO_MODE;++i)autoTick();
  check(autoRun==AutoRun::Ending && !arm.busy() && sortedParts==1 && currentState==AUTO_MODE,9573);
  fakeAckType=CMD_END_STREAM;autoTick();
  check(currentState==TUNING_MENU && tuningReturnState==AUTO_MODE,9574);
  for(float q:commanded)check(q==0,9575);
}
static void profileStorageRoundTripAndLegacyDuplication() {
  SavedRobotSettings s=exampleSavedSettings(),out;
  s.tuning.tall.marginMs+=100;s.tuning.tall.descentMs+=20;s.tuning.tall.closingLeadMs+=10;
  s.tuning.tall.closingDurationMs+=30;s.tuning.tall.openingMs+=40;
  s.tuning.tall.approachZ+=.5f;s.tuning.tall.heightAdjustment-=.5f;
  s.tuning.tall.contactZ+=.5f;s.tuning.tall.partXOffsetMm=23;
  s.tuning.tall.id1ExtraXmm=7.5f;
  uint8_t record[SettingsRecord::size];uint32_t gen;
  SettingsRecord::encode(s,21,record);
  check(SettingsRecord::decode(record,out,gen) && gen==21 && out.velocity==s.velocity && out.homography.valid,9800);
  for(unsigned profile=0;profile<2;++profile)
    for(unsigned row=0;row<PickupTuningMenu::Count;++row)
      check(PickupTuningMenu::value(s.tuning.profile(profile),row)==
            PickupTuningMenu::value(out.tuning.profile(profile),row),9801);
  // Construct schema 2 with its original byte layout, not the current encoder.
  SettingsRecord::put32(record+4,2);SettingsRecord::put32(record+12,104);
  SettingsRecord::put32(record+120,SettingsRecord::crc(record,120));
  SettingsRecord::put32(record+124,SettingsRecord::committed);
  check(SettingsRecord::decode(record,out,gen) && out.velocity==s.velocity && out.homography.valid,9802);
  for(unsigned row=0;row<PickupTuningMenu::Count;++row)
    check(PickupTuningMenu::value(out.tuning.profile(0),row)==
          PickupTuningMenu::value(out.tuning.profile(1),row),9803);
  out.tuning.referenceX=305;out.tuning.testBin=2;
  out.tuning.resetGeneralDefaults();
  check(out.tuning.referenceX==PickMotion::heightReferenceXmm && out.tuning.testBin==0 &&
        out.tuning.partXOffsetMm==15.5f && out.tuning.tall.partXOffsetMm==15.5f,9804);
}
static void trackingAssociatesRawXAcrossProfileChanges() {
  BeltTracker tracker(0);tracker.setVelocity(-100);tracker.setCameraXOffsetsMm(-20,25);
  const calibH c=identityCalibration();objectVision d=calibrationDetection(320,400);
  d.width=40;d.height=20;
  for(unsigned i=0;i<3;++i) {d.yCenter=400-10*i;tracker.updateFrame(&d,1,1000+100*i,c);}
  Part p;check(tracker.candidate(0,p,1200) && p.cameraX==320 && p.x==300,9810);
  const uint32_t id=p.trackId;tracker.reserve(id);
  d.width=20;d.height=40;d.yCenter=370;tracker.updateFrame(&d,1,1300,c);
  check(tracker.observation(id,p) && p.x==345 && p.cameraX==320 && p.boxHeight==40,9811);
  for(unsigned slot=0;slot<MAX_PARTS;++slot)check(!tracker.candidate(slot,p,1300),9812);
  d.width=d.height=30;d.yCenter=360;tracker.updateFrame(&d,1,1400,c);
  check(tracker.observation(id,p) && p.x==300 && p.cameraX==320,9813);
}
static RuntimeTuning fixtureProfiles(Fixture &f) {
  f.config=TrajectoryTest::configuration(f.config,332,f.params,3);
  f.config.closingLeadMs=300;
  RuntimeTuning t;t.profile(0)=PickupTuning(f.config);t.copyLegacyToBoth();
  t.tall.marginMs=1200;t.tall.descentMs=700;t.tall.closingLeadMs=400;
  t.tall.closingDurationMs=650;t.tall.openingMs=80;t.tall.approachZ=56;
  t.tall.heightAdjustment=-2;t.tall.contactZ=-41;t.tall.partXOffsetMm=12;
  return t;
}
static Part profilePart(uint32_t now,bool tall=false) {
  Part p;p.trackId=1;p.x=p.cameraX=332;p.y=400;p.stateMs=now;
  p.boxWidth=tall ? 20:40;p.boxHeight=tall ? 40:20;return p;
}
static void confirmPartClass(Part &p,uint8_t id,uint32_t now) {
  p.classId=p.pendingClass=id;p.classVotes=CameraTracking::ClassificationConfidence::minimumSamples;
  p.classSinceMs=now-CameraTracking::ClassificationConfidence::minimumStableMs;p.classLastMs=now;
}
static void idOneTallCorrectionUsesOnlyConfirmedIdClassification() {
  const AutoConfig base=automaticConfiguration();RuntimeTuning tuning;
  tuning.tall.id1ExtraXmm=12.5f;
  Part p=profilePart(1000,true);p.cameraX=320;p.x=320+tuning.tall.partXOffsetMm;p.y=660;
  confirmPartClass(p,1,1000);AutoPick::Request r;
  const float baseline=p.x;
  check(AutoPick::evaluate(p,-100,base,true,1000,r,&tuning)==AutoPick::Admission::Eligible &&
        r.part.x==baseline+12.5f && r.part.cameraX==320 && r.idExtraXmm==12.5f && r.binId==Classification::binForPartId(1),10200);
  check(fabsf(r.config.pickupZ-PickMotion::beltContactZ(baseline+12.5f,r.config))<.001f &&
        r.profiles.x[1]==baseline+12.5f && r.profiles.x[0]==320+tuning.partXOffsetMm &&
        fabsf(r.profiles.config[1].pickupZ-PickMotion::beltContactZ(r.profiles.x[1],r.profiles.config[1]))<.001f,10201);
  check(p.x==baseline && p.cameraX==320,10202); // Planning does not rewrite tracker coordinates.
  check(AutoPick::evaluate(p,-100,base,false,1000,r,&tuning)==AutoPick::Admission::Eligible &&
        r.part.x==baseline && r.idExtraXmm==0 && r.profiles.x[1]==baseline,10203);
  confirmPartClass(p,partForBin(2),1000);
  check(AutoPick::evaluate(p,-100,base,true,1000,r,&tuning)==AutoPick::Admission::Eligible &&
        r.part.x==baseline && r.idExtraXmm==0 && r.profiles.x[1]==baseline,10204);
  confirmPartClass(p,1,1000);p.boxWidth=40;p.boxHeight=20;p.x=320+tuning.partXOffsetMm;
  check(AutoPick::evaluate(p,-100,base,true,1000,r,&tuning)==AutoPick::Admission::Eligible &&
        r.part.x==p.x && r.idExtraXmm==0 && r.profiles.x[1]==baseline+12.5f,10205);
  const auto synthetic=tuning.plans(base,332,false,1);
  check(synthetic.x[0]==332 && synthetic.x[1]==332,10206);
  p.boxWidth=20;p.boxHeight=40;p.x=baseline;p.classVotes=1;p.classSinceMs=p.classLastMs=1000;
  check(AutoPick::evaluate(p,-100,base,true,1000,r,&tuning)==AutoPick::Admission::AwaitingClassification,10207);
  check(AutoPick::evaluate(p,-100,base,false,1000,r,&tuning)==AutoPick::Admission::Eligible && r.idExtraXmm==0,10208);

  // Admit/reject using the FINAL corrected X, including an initially unknown edge label.
  tuning.tall.id1ExtraXmm=20;p.x=440;p.cameraX=p.x-tuning.tall.partXOffsetMm;
  confirmPartClass(p,1,1000);
  check(AutoPick::evaluate(p,-100,base,true,1000,r,&tuning)==AutoPick::Admission::OutsidePickupRange,10209);
  p.x=190;p.cameraX=p.x-tuning.tall.partXOffsetMm;p.classVotes=1;p.classSinceMs=p.classLastMs=1000;
  check(AutoPick::evaluate(p,-100,base,true,1000,r,&tuning)==AutoPick::Admission::AwaitingClassification,10210);
  confirmPartClass(p,1,1000);
  check(AutoPick::evaluate(p,-100,base,true,1000,r,&tuning)==AutoPick::Admission::Eligible && r.part.x==210,10211);
  confirmPartClass(p,partForBin(2),1000);
  check(AutoPick::evaluate(p,-100,base,true,1000,r,&tuning)==AutoPick::Admission::OutsidePickupRange,10212);
  p.x=332;p.cameraX=p.x-tuning.tall.partXOffsetMm;confirmPartClass(p,1,1000);tuning.tall.id1ExtraXmm=-12.5f;
  check(AutoPick::evaluate(p,-100,base,true,1000,r,&tuning)==AutoPick::Admission::Eligible && r.part.x==319.5f,10213);
}
static void idOneCorrectionSurvivesLiveBoxProfileChangeWithoutChangingIdentity() {
  Fixture f;RuntimeTuning tuning=fixtureProfiles(f);tuning.tall.id1ExtraXmm=10;
  Part p=profilePart(f.now);p.cameraX=332;p.x=332+tuning.partXOffsetMm;p.y=660;
  confirmPartClass(p,1,f.now);AutoPick::Request r;
  check(AutoPick::evaluate(p,-100,f.config,true,f.now,r,&tuning)==AutoPick::Admission::Eligible &&
        f.arm.start(r.part,-100,r.destination,r.config,f.feedback,f.now,&r.profiles) &&
        f.until("SAFETY HEIGHT WAIT"),10220);
  tuning.tall.id1ExtraXmm=90; // Later edits cannot change the frozen assigned plans.
  p.boxWidth=20;p.boxHeight=40;p.classId=partForBin(2);f.arm.updateObservation(p);f.tick();
  check(f.stage("APPROACH") && f.arm.activeProfile()==1,10221);
  const float expectedX=r.profiles.x[1];
  float pose[6];robot_fkine(commanded,f.params,pose);
  check(fabsf(pose[0]-expectedX)<.5f && expectedX==354,10222);
  check(f.until("DESCEND") && f.arm.selectedPickupYaw()==90,10223);
  robot_fkine(commanded,f.params,pose);
  check(fabsf(pose[0]-expectedX)<.5f && fabsf(pose[2]-r.profiles.config[1].pickupZ)<.5f,10224);
  check(f.until("CLASSIFICATION POSE"),10225);robot_fkine(commanded,f.params,pose);
  check(hypotf(hypotf(pose[0]-r.destination[0],pose[1]-r.destination[1]),pose[2]-r.destination[2])<.5f &&
        f.finish()==PickOutcome::Sorted,10226);
}
static void automaticIdOneOffsetIsLimitedToIdClassificationMode() {
  for(unsigned mode=0;mode<3;++mode) {
    setupAutoHarness(-100,false,mode==0);
    if(mode==2)currentState=AUTO_TUNING;
    activeTuning.tall.id1ExtraXmm=12.5f;
    check(startAutoHarness(),10250);Serial.clear();
    feedAutoTrack(332,660,1,-100,0,true);
    check(arm.busy() && autoUntil("DESCEND"),10251);
    float pose[6];robot_fkine(commanded,params,pose);
    const float expectedX=mode==0 ? 344.5f:332;
    check(fabsf(pose[0]-expectedX)<.5f &&
          fabsf(pose[2]-PickMotion::beltContactZ(expectedX,arm.configuration()))<.5f,10252);
    check(strstr(Serial.text,mode==0 ? "idExtraX=+12.50mm":"idExtraX=+0.00mm"),10253);
    for(unsigned i=0;i<400 && !(sortedParts==1 && !arm.busy() && autoRun==AutoRun::Running);++i)autoTick();
    check(sortedParts==1 && !arm.busy() && autoRun==AutoRun::Running && currentState!=EMERGENCY_STOP,10254);
  }
}
static void idOneTallTuningAndSchemaThreeMigrationPreserveCalibrations() {
  setupAutoHarness(-135.14f);settingsFlash.reset();
  new (&settingsStore) SettingsJournal<MemorySettingsFlash>(settingsFlash);
  currentState=AUTO_TUNING;autoTick(0);fakeAckType=CMD_END_STREAM;autoTick();
  check(autoRun==AutoRun::Stopped && PickupTuningMenu::count(0)==9 && PickupTuningMenu::count(1)==10,10230);
  autoTick(20,BTN_L2);pickupTuningRow=PickupTuningMenu::Id1ExtraX;Serial.clear();printPickupTuningRow();
  check(strstr(Serial.text,"10/10 ID 1 TALL extra X") && strstr(Serial.text,"ID classification AUTO only"),10231);
  currentControlCMD.jsLx=1;autoTick();currentControlCMD.jsLx=0;
  check(tuningDraft.tall.id1ExtraXmm==.5f && activeTuning.tall.id1ExtraXmm==0 && tuningDraft.id1ExtraXmm==0,10232);
  autoTick(20,BTN_X);check(activeTuning.tall.id1ExtraXmm==.5f && activeTuning.id1ExtraXmm==0,10233);
  autoTick(20,BTN_L2);check(pickupTuningProfile==0 && pickupTuningRow==PickupTuningMenu::PartXOffset,10234);
  const unsigned positions=positionCommands,grips=gripperCommands;
  autoTick(20,BTN_R1);
  SavedRobotSettings restored;SettingsJournal<MemorySettingsFlash> reboot(settingsFlash);
  check(reboot.load(restored) && restored.tuning.tall.id1ExtraXmm==.5f && restored.tuning.id1ExtraXmm==0 &&
        restored.homography.valid && restored.velocityValid && restored.velocity==-135.14f &&
        positionCommands==positions && gripperCommands==grips,10235);

  SavedRobotSettings source=exampleSavedSettings(),decoded;source.tuning.tall.id1ExtraXmm=25;
  source.tuning.tall.marginMs+=100;uint8_t record[SettingsRecord::size];uint32_t gen;
  SettingsRecord::encode(source,31,record);
  SettingsRecord::put32(record+4,3);SettingsRecord::put32(record+12,140);
  SettingsRecord::put32(record+156,SettingsRecord::crc(record,156));
  check(SettingsRecord::decode(record,decoded,gen) && gen==31 && decoded.tuning.tall.id1ExtraXmm==0 &&
        decoded.tuning.tall.marginMs==source.tuning.tall.marginMs && decoded.tuning.partXOffsetMm==source.tuning.partXOffsetMm &&
        decoded.tuning.tall.partXOffsetMm==source.tuning.tall.partXOffsetMm && decoded.homography.valid &&
        decoded.velocityValid && decoded.velocity==source.velocity,10236);
  MemorySettingsFlash io;io.reset();memcpy(io.bytes,record,sizeof(record));
  SettingsJournal<MemorySettingsFlash> legacy(io);
  check(legacy.load(decoded) && io.erasures==0 && legacy.generation==31,10237);
  decoded.tuning.tall.id1ExtraXmm=-8.5f;
  check(legacy.save(decoded) && legacy.generation==32 && !memcmp(io.bytes,record,sizeof(record)) &&
        SettingsRecord::get32(io.bytes+4096+4)==4,10238);
  SettingsJournal<MemorySettingsFlash> migrated(io);
  check(migrated.load(restored) && restored.tuning.tall.id1ExtraXmm==-8.5f && restored.homography.valid &&
        restored.velocityValid && restored.velocity==source.velocity,10239);
  RuntimeTuning invalid;invalid.tall.id1ExtraXmm=NAN;check(invalid.issue()!=nullptr,10240);
  invalid.tall.id1ExtraXmm=100.5f;check(invalid.issue()!=nullptr,10241);
  invalid.tall.id1ExtraXmm=100;PickupTuningMenu::adjust(invalid.tall,PickupTuningMenu::Id1ExtraX,1);
  check(invalid.tall.id1ExtraXmm==100,10242);
  invalid.tall.id1ExtraXmm=-100;PickupTuningMenu::adjust(invalid.tall,PickupTuningMenu::Id1ExtraX,-1);
  check(invalid.tall.id1ExtraXmm==-100,10243);
  RuntimeTuning shared;TuningMenu::adjust(shared,TuningMenu::Id1TallExtraX,1);
  check(shared.tall.id1ExtraXmm==.5f && TuningMenu::value(shared,TuningMenu::Id1TallExtraX)==.5f &&
        shared.id1ExtraXmm==0,10244);
}
static bool startContactPickup(Fixture &f,bool tall=false,bool profiled=false,bool timed=true) {
  f.config=TrajectoryTest::configuration(f.config,332,f.params,3);
  f.config.pickUpMarginMs=2200;f.config.descentLeadMs=620;f.config.closingLeadMs=360;
  f.config.closingDurationMs=600;f.config.settleMs=250;f.config.minimumMoveMs=300;
  f.config.pickupTimedCompletion=timed;
  RuntimeTuning tuning;tuning.profile(0)=PickupTuning(f.config);tuning.copyLegacyToBoth();
  auto profiles=tuning.plans(f.config,332,false);
  for(AutoConfig &c:profiles.config)c.pickupTimedCompletion=timed;
  const Part p=profilePart(f.now,tall);f.startTime=f.now;
  return f.arm.start(p,-100,f.config.defaultDestination,f.config,f.feedback,f.now,
                     profiled ? &profiles:nullptr);
}
static void timedCompletionIgnoresContactErrorButTimesActualClose() {
  for(unsigned tall=0;tall<2;++tall)for(unsigned profiled=0;profiled<2;++profiled)
    for(unsigned wrap=0;wrap<2;++wrap) {
    Fixture f(wrap ? 0xfffff000U:1000U);
    check(startContactPickup(f,tall,profiled) && f.until("DESCEND"),10000);
    ArmDiagnosticReport descent;
    check(diagnosticOf(f.arm,ArmDiagnosticEvent::DescentCommand,descent) &&
          descent.timedPickup && descent.tallPickup==bool(tall) && descent.postCloseMs==100,10001);
    Print log;printArmDiagnostic(log,descent);
    check(strstr(log.text,tall ? "[PICK POLICY] TALL timed completion":"[PICK POLICY] WIDE timed completion") &&
          strstr(log.text,"600ms closing + 100ms post-close"),10002);
    while(!sawClose && f.arm.busy())f.tick(10,false);
    check(f.stage("CLOSE") && closeTime-f.startTime==3640 && positionCommands==2,10003);
    f.tick(360,false); // Y=0, despite never reaching the gripping pose.
    check(f.stage("CLOSE") && !*f.arm.missReason() &&
          !f.arm.progress(f.feedback,f.now).withinTarget && positionCommands==2,10004);
    f.tick(339,false);
    check(f.stage("CLOSE") && f.now-closeTime==699 && positionCommands==2,10005);
    const float measuredQ3=f.feedback.q[2],measuredQ4=f.feedback.q[3];
    f.tick(1,false);
    check(f.stage("CLASSIFICATION WAYPOINT") && f.now-closeTime==700 &&
          positionCommands==3 && fabsf(commanded[2]-measuredQ3)<.005f &&
          fabsf(commanded[3]-measuredQ4)<.005f,10006);
    ArmDiagnosticReport done;
    check(diagnosticOf(f.arm,ArmDiagnosticEvent::TimedPickupComplete,done) &&
          done.timedPickup && done.tallPickup==bool(tall) && done.closeElapsedMs==700 && done.postCloseMs==100 &&
          !done.progress.withinTarget && !done.hasDeadline && done.remainingArrivalMs==-340,10007);
    log.clear();printArmDiagnostic(log,done);
    check(strstr(log.text,tall ? "TALL_TIMED_COMPLETE":"WIDE_TIMED_COMPLETE") &&
          strstr(log.text,"closeElapsed=700ms requiredClose=700ms postClose=100ms") &&
          strstr(log.text,"grippingPoseCheck=not-required") &&
          !strstr(log.text,"GRIPPING_CONFIRMED"),10008);
    f.tick(500,false); // Waypoint still requires actual position/settling.
    check(f.stage("CLASSIFICATION WAYPOINT") && positionCommands==3,10009);
    check(f.finish()==PickOutcome::Sorted && gripperCommands==2,10010);
  }
  // A late but permitted close command shifts transport from ACTUAL command time.
  for(unsigned tall=0;tall<2;++tall) {
  Fixture jitter;check(startContactPickup(jitter,tall) && jitter.until("DESCEND"),10011);
  jitter.tick(270,false); // Close 10ms after its scheduled deadline.
  check(sawClose && closeTime-jitter.startTime==3650,10012);
  jitter.tick(699,false);check(jitter.stage("CLOSE"),10013);
  jitter.tick(1,false);check(jitter.stage("CLASSIFICATION WAYPOINT"),10014);
  check(jitter.finish()==PickOutcome::Sorted,10015);
  }
}
static void contactAfterArrivalDoesNotCancelTimedPickup() {
  for(unsigned tall=0;tall<2;++tall) {
  Fixture contact;check(startContactPickup(contact,tall) && contact.until("DESCEND"),10020);
  contact.tick(10);contact.tick(250);contact.tick(40);
  check(sawClose && contact.arm.progress(contact.feedback,contact.now).withinTarget &&
        contact.arm.progress(contact.feedback,contact.now).jointHoldActive,10021);
  contact.feedback.q[0]+=3;contact.tick(10,false); // Contact after valid entry.
  check(!*contact.arm.missReason() && contact.stage("CLOSE") &&
        !contact.arm.progress(contact.feedback,contact.now).withinTarget,10022);
  contact.tick(650,false);
  check(contact.stage("CLASSIFICATION WAYPOINT") && contact.finish()==PickOutcome::Sorted,10023);
  }

  // Disabling timed completion still enforces the legacy position gate.
  Fixture tall;check(startContactPickup(tall,true,true,false) && tall.until("DESCEND"),10024);
  while(tall.arm.arrivalDelayMs(tall.now)>50)tall.tick(10,false);
  tall.tick(0);tall.tick(50);
  check(!strcmp(tall.arm.missReason(),"GRIPPING_HEIGHT_LATE") && sawClose,10025);
  ArmDiagnosticReport failure;
  check(diagnosticOf(tall.arm,ArmDiagnosticEvent::Missed,failure) && !failure.timedPickup &&
        failure.progress.withinTarget && failure.progress.stableMs==50 && failure.settleMs==250,10026);
  check(tall.finish()==PickOutcome::Missed && gripperCommands==1 && tall.arm.clawMayHoldPart(),10027);
}
static void timedPickupRetainsFeedbackDeadlinesAndCommandLimits() {
  for(unsigned tall=0;tall<2;++tall) {
  Fixture stale;check(startContactPickup(stale,tall) && stale.until("CLOSE"),10030);
  const unsigned beforeStale=positionCommands;
  stale.tick(stale.config.feedbackTimeoutMs+1,false,false);
  check(stale.finish()==PickOutcome::Fault && !strcmp(stale.arm.fault(),"Nucleo feedback stale") &&
        positionCommands==beforeStale && gripperCommands==1,10031);

  Fixture safety;check(startContactPickup(safety,tall) && safety.until("APPROACH"),10032);
  safety.tick(safety.config.pickUpMarginMs+1,false);
  check(!strcmp(safety.arm.missReason(),"SAFETY_HEIGHT_LATE") && !sawClose &&
        safety.finish()==PickOutcome::Missed,10033);

  Fixture late;check(startContactPickup(late,tall) && late.until("DESCEND"),10034);
  late.tick(late.config.descentLeadMs-late.config.closingLeadMs+51,false);
  check(!strcmp(late.arm.missReason(),"CLOSE_COMMAND_LATE") && !sawClose &&
        late.finish()==PickOutcome::Missed,10035);

  Fixture limits;check(startContactPickup(limits,tall) && limits.until("CLOSE"),10036);
  limits.feedback.q[2]=limits.params.q_max[2]*180/PI+1;
  const unsigned beforeLimits=positionCommands;
  limits.tick(700,false);
  check(limits.finish()==PickOutcome::Fault && !strcmp(limits.arm.fault(),"Joint target outside limits") &&
        positionCommands==beforeLimits && gripperCommands==1,10037);

  Fixture timeout;check(startContactPickup(timeout,tall) && timeout.until("DESCEND"),10038);
  // Delay all closing completion beyond the normal, still-active stage watchdog.
  timeout.tick(270,false);check(sawClose,10039);
  timeout.tick(timeout.config.motionTimeoutMs,false);
  check(timeout.finish()==PickOutcome::Fault && !strcmp(timeout.arm.fault(),"Stage timeout"),10040);
  }
  AutoConfig invalid;invalid.pickupPostCloseMs=60000;
  check(!validPickupTiming(invalid),10041);
}
static void timedPauseAndCancellationKeepTheClawClosed() {
  for(unsigned tall=0;tall<2;++tall) {
  Fixture paused;check(startContactPickup(paused,tall) && paused.until("DESCEND"),10050);
  while(!sawClose && paused.arm.busy())paused.tick(10,false);
  paused.tick(100,false);paused.arm.requestPause(paused.now);
  paused.tick(599,false);check(!paused.arm.paused() && paused.stage("CLOSE"),10051);
  paused.tick(1,false);
  check(paused.arm.paused() && paused.stage("CLOSE") && positionCommands==2,10052);
  paused.tick(20000,false);check(paused.arm.paused() && paused.arm.busy(),10053);
  check(paused.arm.resume(paused.now),10054);paused.tick(10,false);
  check(paused.stage("CLASSIFICATION WAYPOINT") && paused.finish()==PickOutcome::Sorted,10055);

  Fixture beforeClose;check(startContactPickup(beforeClose,tall) && beforeClose.until("DESCEND"),10056);
  beforeClose.arm.requestPause(beforeClose.now);beforeClose.tick(300,false);
  check(!beforeClose.arm.paused() && !sawClose,10057);
  beforeClose.tick(0);beforeClose.tick(250);
  check(beforeClose.arm.paused() && !sawClose,10058);
  beforeClose.tick(15000);check(beforeClose.arm.resume(beforeClose.now),10059);
  check(beforeClose.until("CLOSE") && beforeClose.finish()==PickOutcome::Sorted,10060);

  Fixture cancelled;check(startContactPickup(cancelled,tall) && cancelled.until("CLOSE"),10061);
  cancelled.arm.cancelPickup(cancelled.feedback,cancelled.now);
  cancelled.tick(699,false);check(cancelled.stage("RECOVERY CLOSE WAIT") && gripperCommands==1,10062);
  cancelled.tick(1,false);check(cancelled.stage("MISSED-PICK RECOVERY"),10063);
  check(cancelled.finish()==PickOutcome::Missed && gripperCommands==1 &&
        cancelled.arm.clawMayHoldPart() && !strcmp(cancelled.arm.missReason(),"OPERATOR_CANCELLED"),10064);
  }
}
static void automaticTallContactFinishesByTimeAndReturnsHome() {
  setupAutoHarness(-100);
  activeTuning.tall.marginMs=2200;activeTuning.tall.descentMs=620;
  activeTuning.tall.closingLeadMs=360;activeTuning.tall.closingDurationMs=600;
  check(startAutoHarness(),10070);
  feedAutoTrack(332,660,partForBin(3),-100,0,true);
  check(autoUntil("DESCEND") && arm.selectedPickupYaw()==90 &&
        arm.configuration().closingDurationMs==600,10071);
  while(!sawClose && arm.busy() && currentState==AUTO_MODE)autoTick(20,0,false);
  const uint32_t actualClose=adapterTime;
  const unsigned beforeTransport=positionCommands;
  check(sawClose && autoStage("CLOSE"),10072);
  autoTick(699,0,false);
  check(currentState==AUTO_MODE && autoStage("CLOSE") && !*arm.missReason() &&
        positionCommands==beforeTransport && !arm.progress(armFeedback,adapterTime).withinTarget,10073);
  autoTick(1,0,false);
  check(autoStage("CLASSIFICATION WAYPOINT") && adapterTime-actualClose==700 &&
        positionCommands==beforeTransport+1,10074);
  for(unsigned i=0;i<300 && !(sortedParts==1 && !arm.busy() && autoRun==AutoRun::Running);++i)autoTick();
  check(currentState==AUTO_MODE && sortedParts==1 && auto_parts_missed==0 &&
        !arm.busy() && autoRun==AutoRun::Running && !arm.clawMayHoldPart(),10075);
  for(float q:commanded)check(q==0,10076);
}
static void profilesFollowLastBoxAndFreezeAtDescent() {
  Fixture f;RuntimeTuning t=fixtureProfiles(f);
  const PickupProfilePlan plans=t.plans(f.config,332,false);
  Part p=profilePart(f.now);f.startTime=f.now;
  check(f.arm.start(p,-100,f.config.defaultDestination,plans.config[0],f.feedback,f.now,&plans) &&
        f.until("WAIT FOR APPROACH"),9820);
  p.boxWidth=20;p.boxHeight=40;f.arm.updateObservation(p);f.tick();
  check(f.arm.activeProfile()==1 && f.arm.configuration().pickUpBudgetMs()==1900,9821);
  // The controller owns both immutable profiles, not the caller's edit buffer.
  t.tall.marginMs=5000;
  check(f.until("APPROACH") && f.now-f.startTime==2100,9822);
  float pose[6];robot_fkine(commanded,f.params,pose);
  check(fabsf(pose[0]-332)<.5f && fabsf(pose[2]-56)<.5f,9823);
  check(f.until("DESCEND") && f.now-f.startTime==3300 && f.arm.selectedPickupYaw()==90,9824);
  robot_fkine(commanded,f.params,pose);
  check(fabsf(pose[2]-plans.config[1].pickupZ)<.5f && f.arm.configuration().closingDurationMs==650,9825);
  p.boxWidth=40;p.boxHeight=20;f.arm.updateObservation(p);f.tick();
  check(f.arm.activeProfile()==1 && f.arm.configuration().openingDurationMs==80,9826);
  check(f.until("CLASSIFICATION WAYPOINT") && closeTime-f.startTime==3600 &&
        f.now-closeTime==750,9827); // TALL's own 650ms closing duration + 100ms.
  check(f.finish()==PickOutcome::Sorted && gripperCommands==2,9828);
}
static void profileSwitchReconfirmsSafetyOrRecoversWhenLate() {
  Fixture f;RuntimeTuning t=fixtureProfiles(f);const auto plans=t.plans(f.config,320,true);
  Part p=profilePart(f.now);p.x=plans.x[0];p.cameraX=320;
  check(f.arm.start(p,-100,f.config.defaultDestination,plans.config[0],f.feedback,f.now,&plans) &&
        f.until("SAFETY HEIGHT WAIT"),9830);
  const unsigned commands=positionCommands;
  p.boxWidth=20;p.boxHeight=40;f.arm.updateObservation(p);f.tick();
  check(f.stage("APPROACH") && positionCommands==commands+1 && f.arm.activeProfile()==1,9831);
  float pose[6];robot_fkine(commanded,f.params,pose);
  check(fabsf(pose[0]-332)<.5f && fabsf(pose[2]-56)<.5f &&
        !f.arm.progress(f.feedback,f.now).withinTarget,9832);
  check(f.until("SAFETY HEIGHT WAIT") && f.finish()==PickOutcome::Sorted,9833);

  Fixture late;t=fixtureProfiles(late);auto bad=t.plans(late.config,332,false);
  bad.config[1].descentLeadMs=1600;
  p=profilePart(late.now);
  check(late.arm.start(p,-100,late.config.defaultDestination,bad.config[0],late.feedback,late.now,&bad) &&
        late.until("SAFETY HEIGHT WAIT"),9834);
  p.boxWidth=20;p.boxHeight=40;late.arm.updateObservation(p);late.tick();
  check(!strcmp(late.arm.missReason(),"BOX_PROFILE_CHANGED_LATE") && !sawClose,9835);
  check(late.finish()==PickOutcome::Missed && !late.arm.clawMayHoldPart(),9836);
  for(float q:commanded)check(q==0,9837);
}
static void profileAvailabilityAndAdmissionArePerRatio() {
  Fixture f;RuntimeTuning t=fixtureProfiles(f);
  t.marginMs=2000;Part p=profilePart(f.now);p.x=332;p.cameraX=332-t.partXOffsetMm;p.y=220;
  AutoPick::Request r;
  check(AutoPick::evaluate(p,-100,f.config,false,f.now,r,&t)==AutoPick::Admission::TooLate,9840);
  p.boxWidth=20;p.boxHeight=40;
  check(AutoPick::evaluate(p,-100,f.config,false,f.now,r,&t)==AutoPick::Admission::Eligible &&
        r.config.pickUpBudgetMs()==1900 && r.profiles.x[1]==p.cameraX+t.tall.partXOffsetMm,9841);
  t=fixtureProfiles(f);auto plans=t.plans(f.config,332,false);plans.available[1]=false;
  p=profilePart(f.now);
  check(f.arm.start(p,-100,f.config.defaultDestination,plans.config[0],f.feedback,f.now,&plans) &&
        f.until("WAIT FOR APPROACH"),9842);
  p.boxWidth=20;p.boxHeight=40;f.arm.updateObservation(p);f.tick();
  check(!strcmp(f.arm.missReason(),"BOX_PROFILE_UNAVAILABLE") &&
        f.finish()==PickOutcome::Missed && !sawClose,9843);
}
static void autoPickupTuningEditsOnlyIdleAndSavesStopped() {
  setupAutoHarness(-100,true,true);menu_option1=1;
  autoTuningLaunchPending=true;currentState=WAITING_ALGO_SWITCH;lastState=CALIBRATION_MENU;
  requestedTagsAlgo=requestedAlgoVelocity=false;
  cameraAlgorithm.begin(CUSTOM_MODEL1_ID);cameraAlgorithm.acknowledge();cameraAlgorithm.switched();
  serviceAlgorithmSelection();
  check(currentState==AUTO_TUNING && menu_option0==1 && menu_option1==1 && menu_option2==1 &&
        autoMovesArm() && !autoRespectsId() && controllerLossRequiresStop(),9850);
  check(startAutoHarness(),9851);
  const auto wide=activeTuning.profile(0);const auto tall=activeTuning.tall;
  autoTick(1,BTN_L2);currentControlCMD.jsLx=1;autoTick(1);
  check(tuningDraft.tall.marginMs==tall.marginMs+10 && activeTuning.tall.marginMs==tall.marginMs &&
        tuningDraft.marginMs==wide.marginMs,9852);
  currentControlCMD.jsLx=0;autoTick(1,BTN_X);
  check(activeTuning.tall.marginMs==tall.marginMs+10 && homographyDone && velocityCalibrated,9853);
  feedAutoTrack(332,660,99,-100,3); // ID ignorant must pick an otherwise unmapped label.
  check(arm.busy() && autoStage("PLAN"),9854);
  const auto draft=tuningDraft.tall;
  currentControlCMD.jsLx=1;
  for(unsigned i=0;i<700 && arm.busy() && currentState==AUTO_TUNING;++i)autoTick(20,BTN_X|BTN_R1|BTN_L2);
  check(!arm.busy() && currentState==AUTO_TUNING && sortedParts==1 && !pickupSaveRequested &&
        tuningDraft.tall.marginMs==draft.marginMs && pickupTuningProfile==1,9855);
  currentControlCMD.jsLx=0;autoTick();
  settingsFlash.reset();new (&settingsStore) SettingsJournal<MemorySettingsFlash>(settingsFlash);
  tuningDraft.tall.partXOffsetMm+=.5f;
  autoTick(20,BTN_R1);
  check(pickupSaveRequested && stopRequested && settingsStore.generation==0,9856);
  autoTick();check(autoRun==AutoRun::Ending && lastCriticalType==CMD_END_STREAM,9857);
  fakeAckType=CMD_END_STREAM;const unsigned cmds=positionCommands;
  autoTick();
  check(autoRun==AutoRun::Stopped && settingsStore.generation==1 && !streamStarted && !arm.busy() &&
        !strcmp(Serial1.batch,"\nS:MODES\n") && positionCommands==cmds,9858);
  SavedRobotSettings loaded;SettingsJournal<MemorySettingsFlash> reboot(settingsFlash);
  check(reboot.load(loaded) && loaded.tuning.tall.partXOffsetMm==activeTuning.tall.partXOffsetMm &&
        loaded.velocityValid && loaded.homography.valid,9859);
  autoTick(20,BTN_L1);check(currentState==CALIBRATION_MENU,9860);
}
static void fixedShoulderClassificationEnforcesFullPoseForBothBoxProfiles() {
  for(unsigned tall=0;tall<2;++tall)for(unsigned bin=1;bin<=3;++bin) {
    Fixture f;f.config=TrajectoryTest::configuration(f.config,332,f.params,bin);
    RuntimeTuning tuning;tuning.profile(0)=PickupTuning(f.config);tuning.copyLegacyToBoth();
    const auto profiles=tuning.plans(f.config,332,false);
    Part p=profilePart(f.now,tall);
    check(f.config.classificationFixedShoulders && !f.config.holdPickupQ5ForSorting &&
      f.arm.start(p,-100,f.config.defaultDestination,profiles.config[tall],f.feedback,f.now,&profiles) &&
      f.until("DESCEND"),9900);
    ClassificationPlanReport r;
    check(f.arm.takeClassificationPlan(r) && r.fixedShoulders && !r.heldPickupQ5 &&
      r.ik.task==IkTask::FullPose && r.ik.positionErrorMm<=.5f && r.ik.fullRotationErrorRad<=.035f,9901);
    check(!f.arm.takeClassificationPlan(r),9902);
    const float angle=90+Classification::bins[bin-1].rollOffsetDegrees;
    check(r.joints[0]==0 && fabsf(r.joints[1]-angle)<.01f &&
      fabsf(r.planned[3]-Classification::referencePose[3])<2.5f &&
      fabsf(r.planned[4]-Classification::referencePose[4])<2.5f &&
      fabsf(r.planned[5]-angle)<2.5f,9903);
    Serial.clear();printClassificationPlan(Serial,r);
    check(strstr(Serial.text,"task=full pose") && strstr(Serial.text,"q5=free") &&
      strstr(Serial.text,"requested TCP=") && strstr(Serial.text,"planned TCP="),9904);
    const float pickupQ5=commanded[4],pickupQ4=commanded[3];
    check(f.until("CLASSIFICATION WAYPOINT") && fabsf(commanded[3]-pickupQ4)<.005f &&
          fabsf(commanded[4]-pickupQ5)<.005f,9905);
    check(f.until("CLASSIFICATION POSE") && commanded[0]==0 &&
      fabsf(commanded[1]-angle)<.01f && fabsf(commanded[4])<1,9906);
    if(tall)check(fabsf(commanded[4]-pickupQ5)>80,9907); // Released only at final classification.
    check(f.finish()==PickOutcome::Sorted,9908);
    check(f.arm.prepare(f.config,f.feedback,f.now) && !f.arm.takeClassificationPlan(r),9909);
  }
}
static void unreachableOrientationCannotChangeShouldersOrPositionSilently() {
  for(unsigned profiled=0;profiled<2;++profiled) {
    Fixture f;f.config=TrajectoryTest::configuration(f.config,332,f.params,3);
    // Original XYZ is incompatible with the requested tilt when shoulders are fixed.
    f.config.defaultDestination[0]=-76;f.config.defaultDestination[1]=323;f.config.defaultDestination[2]=170;
    RuntimeTuning tuning;tuning.profile(0)=PickupTuning(f.config);tuning.copyLegacyToBoth();
    const auto profiles=tuning.plans(f.config,332,false);Part p=profilePart(f.now,true);
    check(f.arm.start(p,-100,f.config.defaultDestination,f.config,f.feedback,f.now,
                     profiled ? &profiles:nullptr),9910);
    check(f.finish()==PickOutcome::Rejected && positionCommands==0 && gripperCommands==0,9911);
  }
  Fixture locked;locked.config=TrajectoryTest::configuration(locked.config,332,locked.params,3);
  locked.config.holdPickupQ5ForSorting=true;
  check(locked.pick() && locked.finish()==PickOutcome::Rejected && positionCommands==0,9912);
  Fixture compatible;compatible.config=TrajectoryTest::configuration(compatible.config,332,compatible.params,3);
  compatible.config.holdPickupQ5ForSorting=true;
  RuntimeTuning compatibleTuning;compatibleTuning.profile(0)=PickupTuning(compatible.config);
  compatibleTuning.copyLegacyToBoth();auto compatiblePlans=compatibleTuning.plans(compatible.config,332,false);
  compatiblePlans.available[1]=false; // Only yaw0 is compatible with this optional q5 lock.
  Part wide=profilePart(compatible.now,false);
  check(compatible.arm.start(wide,-100,compatible.config.defaultDestination,compatible.config,
       compatible.feedback,compatible.now,&compatiblePlans) && compatible.until("DESCEND"),9913);
  ClassificationPlanReport held;
  check(compatible.arm.takeClassificationPlan(held) && held.heldPickupQ5 &&
       fabsf(held.joints[4]-commanded[4])<.005f && compatible.finish()==PickOutcome::Sorted,9914);
}
static void analyticClassificationWorkspaceMatchesFirmwareFk() {
  const RobotDHParams params=robot_get_dh_params();
  const float pitch=Classification::referencePose[4],angle=pitch*PI/180;
  const float centerR=47.45f-164.14f*sinf(angle),centerZ=418.022f-164.14f*cosf(angle);
  const float minimum=fmaxf(params.q_min[2]*180/PI,params.q_min[3]*180/PI-pitch);
  const float maximum=fminf(params.q_max[2]*180/PI,params.q_max[3]*180/PI-pitch);
  const float elbows[]={minimum,minimum*.75f,minimum*.5f,minimum*.25f,0,maximum*.5f,maximum};
  for(unsigned bin=0;bin<3;++bin)for(float elbow:elbows) {
    const float roll=90+Classification::bins[bin].rollOffsetDegrees;
    float q[]={0,roll,elbow,elbow+pitch,0},tcp[6];robot_fkine(q,params,tcp);
    const float r=centerR+307*cosf(elbow*PI/180),z=centerZ+307*sinf(elbow*PI/180);
    const float x=-76+r*cosf(roll*PI/180),y=r*sinf(roll*PI/180);
    check(hypotf(hypotf(tcp[0]-x,tcp[1]-y),tcp[2]-z)<.001f &&
      fabsf(tcp[3])<.001f && fabsf(tcp[4]-pitch)<.001f && fabsf(tcp[5]-roll)<.001f,9920);
    for(unsigned j=0;j<5;++j)
      check(q[j]*PI/180>=params.q_min[j]-1e-6f && q[j]*PI/180<=params.q_max[j]+1e-6f,9921);
  }
}
#define RUN_TEST(name) do { snprintf(test_context, sizeof(test_context), "%s", #name); name(); } while (0)
extern "C" void test_main() {
#ifdef TEST_CLASSIFICATION_ONLY
  RUN_TEST(fixedShoulderClassificationEnforcesFullPoseForBothBoxProfiles);
  RUN_TEST(unreachableOrientationCannotChangeShouldersOrPositionSilently);
  RUN_TEST(analyticClassificationWorkspaceMatchesFirmwareFk);
  RUN_TEST(heldQ5MustStillPreserveClassificationPosition);
  test_done();return;
#endif
#ifndef TEST_VISION_DIAGNOSTICS_ONLY
  RUN_TEST(idOneTallCorrectionUsesOnlyConfirmedIdClassification);
  RUN_TEST(idOneCorrectionSurvivesLiveBoxProfileChangeWithoutChangingIdentity);
  RUN_TEST(automaticIdOneOffsetIsLimitedToIdClassificationMode);
  RUN_TEST(idOneTallTuningAndSchemaThreeMigrationPreserveCalibrations);
  RUN_TEST(autoCounterResetRequiresStoppedAndPreservesSettings);
  RUN_TEST(timedCompletionIgnoresContactErrorButTimesActualClose);
  RUN_TEST(contactAfterArrivalDoesNotCancelTimedPickup);
  RUN_TEST(timedPickupRetainsFeedbackDeadlinesAndCommandLimits);
  RUN_TEST(timedPauseAndCancellationKeepTheClawClosed);
  RUN_TEST(automaticTallContactFinishesByTimeAndReturnsHome);
  RUN_TEST(fixedShoulderClassificationEnforcesFullPoseForBothBoxProfiles);
  RUN_TEST(unreachableOrientationCannotChangeShouldersOrPositionSilently);
  RUN_TEST(analyticClassificationWorkspaceMatchesFirmwareFk);
  RUN_TEST(profileStorageRoundTripAndLegacyDuplication);
  RUN_TEST(trackingAssociatesRawXAcrossProfileChanges);
  RUN_TEST(profilesFollowLastBoxAndFreezeAtDescent);
  RUN_TEST(profileSwitchReconfirmsSafetyOrRecoversWhenLate);
  RUN_TEST(profileAvailabilityAndAdmissionArePerRatio);
  RUN_TEST(autoPickupTuningEditsOnlyIdleAndSavesStopped);
  RUN_TEST(settingsCodecAndJournalSurviveInterruptedWrites);
  RUN_TEST(legacySettingsRetainTuningAndCalibrationsWhenOffsetIsAdded);
  RUN_TEST(storageLayoutNeverOverlapsExistingPartitions);
  RUN_TEST(runtimeTuningUsesExactStepsAndSharedGeometry);
  RUN_TEST(controllerMenuAppliesSavesReloadsAndKeepsCalibrationsIndependent);
  RUN_TEST(runtimeCameraOffsetChangesTrackingAdmissionAndVisionDiagnosticsTogether);
  RUN_TEST(automaticTuningRequestFinishesCurrentCycleAndStops);
  RUN_TEST(wristHoldRequiresConfirmedEntryAndResetsForEveryMove);
  RUN_TEST(wristHoldingStillRequiresTcpOtherJointsAndFreshFeedback);
  RUN_TEST(missDiagnosticsRetainFailureTargetAndActualSettings);
  RUN_TEST(grippingMissSeparatesSettlingFromDescentDeadline);
  RUN_TEST(confirmationDiagnosticsShowHeadroomAndDoNotConsumeEventsTwice);
  RUN_TEST(scheduledPickupAndConcurrentClosing);
  RUN_TEST(missedPickupReturnsHomeWithoutRelease);
  RUN_TEST(scheduledBoundaryJitterAndClockWrap);
  RUN_TEST(trackingRectangleAndReservedLastBox);
  RUN_TEST(autoUsesReservedLastBoxAndStopsIfRecoveryHoldsPart);
#ifndef TEST_SETTINGS_ONLY
  RUN_TEST(velocityRegionFiltersWholeBoxes);
  RUN_TEST(sparseMovingTagIgnoresDuplicateCornerIds);
  RUN_TEST(movingTagStreamPreservesAllDetections);
  RUN_TEST(heldTagPositionsAreNotIndependentSamples);
  RUN_TEST(controllerLossStopsOnlyMotionModes);
  RUN_TEST(actualTagCalibrationRequiresIndependentPasses);
  RUN_TEST(actualTagCalibrationWaitsRetriesAndCancels);
  RUN_TEST(algorithmSelectionReusesOnlyConfirmedCurrentModel);
  RUN_TEST(jointMotionRepeatedRunAndLoggingHandshake);
  RUN_TEST(jointMotionCancelAndFailures);
  RUN_TEST(newPartsWhileSortingDoNotReplaceTheAssignedSnapshot);
  RUN_TEST(transientFirstLabelDoesNotChooseTheBin);
  RUN_TEST(classificationNeedsObservedSpanNotHeldPackets);
  RUN_TEST(uncertainClassExpiresWithoutStartingTheArm);
  RUN_TEST(normalRun);
  RUN_TEST(pauseMovingAndFreezeTime);
  RUN_TEST(pauseCountdown);
  RUN_TEST(pauseClosing);
  RUN_TEST(pausedFeedbackStillRequired);
  RUN_TEST(cancelPausedAndHome);
  RUN_TEST(homePauseAndClockWrap);
  RUN_TEST(conservativeAdmissionBudgetIsRespected);
  RUN_TEST(lostPausedPose);
  RUN_TEST(pausedPartClockAcrossWrap);
  RUN_TEST(homeCommandBatch);
  RUN_TEST(homeTimeoutDiagnostics);
  RUN_TEST(recoveryAfterSlowApproach);
  RUN_TEST(jointToleranceDoesNotGuaranteeTcpTolerance);
  RUN_TEST(quantizedReachablePoses);
  RUN_TEST(manualProjectionAndTransactionalTarget);
  RUN_TEST(realisticHomeAndRejectionDiagnostics);
  RUN_TEST(repeatedCartesianTargetsAndLeftDriveLoss);
  RUN_TEST(calibratedTrajectoryTestRange);
  RUN_TEST(jointLimitCommandPrecision);
  RUN_TEST(heldQ5MustStillPreserveClassificationPosition);
  RUN_TEST(binGeometryAndPartRouting);
  RUN_TEST(waypointHoldsMeasuredQ4);
  RUN_TEST(invalidFixedJointSeedIsRejected);
  RUN_TEST(finalHomePauseAndFeedbackSupervision);
  RUN_TEST(tunedHeightAndHomeWristAllowance);
  RUN_TEST(arrivalHysteresisEntryResetAndFreshSamples);
  RUN_TEST(waypointLogReplayAndStrictFinalDropTcp);
  RUN_TEST(pausedHysteresisStillDetectsDriftAndStaleFeedback);
  RUN_TEST(invalidArrivalSettingsNeverMove);
  RUN_TEST(cameraRotationKeepsWorldAxes);
  RUN_TEST(velocityCalibrationRejectionReasons);
  RUN_TEST(velocitySampleThrottlingAndClockWrap);
  RUN_TEST(calibrationRmsTrialAcceptsModerateScatterOnly);
  RUN_TEST(approachWristDitherNoLongerBlocksTest);
  RUN_TEST(strictWristLimitReproducesReportedTimeout);
  RUN_TEST(recoveryWristAllowanceStillChecksTcpAndJointError);
  RUN_TEST(automaticAdmissionUsesRealSnapshotsAndSharedGeometry);
  RUN_TEST(automaticRealCameraCycles);
  RUN_TEST(automaticStopCompletesHomeBeforeStreamStop);
  RUN_TEST(automaticTrackingContinuesDuringReturn);
  RUN_TEST(automaticVisionOnlyAndAdmissionGuards);
  RUN_TEST(automaticSupervisionAndEarlyStop);
  RUN_TEST(planningConsumesItsAdmissionBudgetOnlyOnce);
  RUN_TEST(automaticMissedPickReturnsHomeAndControllerLossStopsReturn);
  RUN_TEST(partMeasurementsNeverChangeCalibratedSpeed);
  RUN_TEST(automaticReadinessReportsActualBlocker);
  RUN_TEST(cameraPartOffsetPrecedesTrackingAndAdmission);
#endif
#endif
  RUN_TEST(visionDiagnosticsWithoutArmMotion);
  RUN_TEST(diagnosticQueueSurvivesBusyAndShortWrites);
  test_done();
}
