#include <BasicLinearAlgebra.h>
#include <WiFi.h>
#include <WiFiUDP.h>
using namespace BLA;
#include "BeltTracker.h"
#include "BeltCalibrationSession.h"
#include "ArmTrajectory.h"
#include "ArmTrajectoryDiagnostics.h"
#include "PortentaSettingsFlash.h"
#include "TrajectoryTestConfig.h"
#include "AutoPickSelection.h"
#include "NonBlockingVisionUdp.h"
#include "ControllerLinkDiagnostics.h"
#include "CameraAlgorithmSelection.h"
#include "JointCalibrationWizard.h"
#include "JointMotionTest.h"
#include "DOF5_wServo_Kinematics.h"
#include "PS3DataPacket.h"
#include "VisionPacket.h"
#include <math.h>

// Defined with the state supervisor below; also used by early link diagnostics.
bool controllerLossRequiresStop();

// ===================================
// Homography variables
// ===================================

calibH camera_calib;

// ===================================
// Vision Tracking defines
// ===================================

#define CUSTOM_MODEL2_ID 129
#define APRIL_TAG_ALGO_ID 16
#define CUSTOM_MODEL1_ID 131

BeltTracker mainTracker;

AutoConfig autoConfig;
ArmFeedback armFeedback;
ArmTrajectory arm;
RuntimeTuning activeTuning, tuningDraft;
PortentaSettingsFlash settingsFlash;
SettingsJournal<PortentaSettingsFlash> settingsStore(settingsFlash);
bool tuningMenuRequested=false;
unsigned tuningRow=0;
int8_t tuningLastX=0,tuningLastY=0;
uint32_t tuningRepeatMs=0;
bool autoTuningLaunchPending=false,pickupSaveRequested=false,pickupEditorLocked=false;
bool pickupEditorNeedsNeutral=false;
unsigned pickupTuningProfile=0,pickupTuningRow=0;
int8_t pickupLastX=0,pickupLastY=0;
uint32_t pickupRepeatMs=0;
bool applyTuningDraft();
bool servicePickupTuning(uint32_t now,uint16_t pressed);
void savePickupTuning();
void printPickupTuningRow();
JointCalibrationWizard jointWizard;
JointMotionTest jointMotionTest;
uint32_t jointCalibrationEntryMs = 0;
bool jointCalibrationEntryHeld = false;
BeltCalibrator beltCalibration;
BeltCalibrationSession beltCalibrationSession;
uint32_t visionTimestampMs = 0, taskStartedMs = 0;
uint32_t lastVelocityCalibrationLogMs = 0;
uint8_t lastAckType = 0;
bool switchAcked = false, stopRequested = false, exitRequested = false;
enum class AutoRun : uint8_t { SyncStop, Stopped, Starting, Preparing, Running, ReturningHome, Ending };
AutoRun autoRun = AutoRun::Stopped;
const char *autoLastStage = "";
enum class CalPhase : uint8_t { Starting, Collecting, Ending };
CalPhase calPhase = CalPhase::Starting;
uint32_t sortedParts = 0, rejectedParts = 0, ignoredParts = 0;
uint32_t outsidePickupParts = 0;
bool discardNucleoLine = false;

// Camera-independent dry run; negative Y is the conveyor direction.
float trajectoryTestXmm = TrajectoryTest::minimumXmm;
uint8_t trajectoryTestPartId = 0, trajectoryTestBinId = 3;
bool trajectoryTestTallPart = true;
enum class TrajectoryTestRun : uint8_t { Ready, Preparing, Picking, FinishingHome, ReturningHome };
TrajectoryTestRun trajectoryTestRun = TrajectoryTestRun::Ready;
AutoConfig trajectoryTestConfig;
PickupProfilePlan trajectoryTestProfiles;
const char *testLastStage = "";
bool testLastPaused = false;
uint32_t lastArmProgressMs = 0;

// ===================================
// WIFI variables
// ===================================
const char *ssid = "Red_Portenta";
const char *password = "robotica_uvg";
IPAddress esp32IP;

bool esp32IP_known = false;

// ===================================
// UDP variables
// ===================================
// Controller UDP
WiFiUDP udpController;
const uint16_t udpPort_Controller = 4210;
uint32_t lastSeqReceived = 0;
unsigned long lastTimeReceived = 0;
const unsigned long TIMEOUT_FAILSAFE_MS = 350;
bool udpControllerStable = false;
ControllerLinkDiagnostics controllerLink;
uint16_t controllerPressedEdges = 0;
bool controllerEmergencyReceived = false, controllerTimedOutThisLoop = false;

// Camera UDP
NonBlockingVisionUdp cameraUDP;
const uint16_t cameraUDPport = 4220; // udpPort_Camera = 4220;

criticalMessage pendingESPmsg;
uint8_t globalMsgCounter = 1;
const unsigned long RETRY_TIMEOUT_MS = 100;
const uint8_t MAX_TRIES = 5;
bool messageAck = false;

unsigned long nextResyncTry = 0;
const unsigned long RESYNC_TIME_DELAY_MS = 3000;
uint8_t commFails = 0;
const uint8_t MAX_COMM_FAILS = 3;
bool communicationError = false;
bool packetMissed = false;

uint8_t totalObjects = 0;

// ===================================
// Vision variables
// ===================================
objectVision latestReceived[10];

bool cameraOffline = true;

bool homographyDone = false;
bool cameraCancelRequested=false;

bool velocityCalibrated = false;

bool requestedTagsAlgo = false;

bool requestedCustomAlgo = false;

bool requestedAlgoVelocity = false;

bool algoSwitched = false;
CameraAlgorithmSelection cameraAlgorithm;

bool invalidAlgo = false;

bool newLatest = false;

bool streamStarted = false;
int auto_parts_missed = 0;

// ===================================
// PS3 Control variables
// ===================================
enum PS3ButtonsMask {
  BTN_B = 1 << 0,  // 0x0001 (A)
  BTN_A = 1 << 1,  // 0x0002 (B)
  BTN_Y = 1 << 2,  // 0x0004 (X)
  BTN_X = 1 << 3,  // 0x0008 (Y)
  BTN_L1 = 1 << 4, // 0x0010
  BTN_R1 = 1 << 5, // 0x0020
  BTN_L2 = 1 << 6, // 0x0040
  BTN_R2 = 1 << 7, // 0x0080
  BTN_L3 = 1 << 8, // 0x0100 (ThumbL)
  BTN_R3 = 1 << 9  // 0x0200 (ThumbR)
};
PS3ControlPacket currentControlCommand;
bool newControlCMD = false;
struct ControlParsedCMD {
  uint16_t buttons = 0;
  // bool btnX=false, btnY=false, btnB=false, btnA=false;
  // bool btnL1=false, btnL2=false, btnR1=false, btnR2=false;
  // bool btnL3=false, btnR3=false;
  int8_t jsLx = 0, jsLy = 0, jsRx = 0, jsRy = 0;

  bool operator==(const ControlParsedCMD &other) const {
    return (buttons == other.buttons && jsLx == other.jsLx && jsLy == other.jsLy &&
            jsRx == other.jsRx && jsRy == other.jsRy);
  }
};
ControlParsedCMD currentControlCMD;
ControlParsedCMD lastControlCMD;

const uint8_t joystick_deadzone = 100;

bool leave_ES = false;
bool entered_ES = false;
bool holdingResetCombo = false;
bool escapeCombo = false;
unsigned long resetComboStartTime = 0;
uint16_t PE_buttons;
int8_t last_move[7] = {0, 0, 0, 0, 0, 0, 0};

uint16_t global_velocity = 500;

unsigned long lastStepTime = 0;
const unsigned long TARGET_STEP_DELAY = 100;

// unsigned long lastSpeedChangeTime = 0;
// const unsigned long SPEED_CHANGE_DELAY_MS = 200;

// ===================================
// Kinematics variables
// ===================================
bool limits_received = false;
float LIMIT_ROLL_MIN = -60.0f, LIMIT_ROLL_MAX = 120.0f;
float LIMIT_PITCH_MIN = -50.0f, LIMIT_PITCH_MAX = 60.0f;
float LIMIT_DOF3_MIN = -70.0f, LIMIT_DOF3_MAX = 90.0f;
float LIMIT_DOF4_MIN = -200.0f, LIMIT_DOF4_MAX = 30.0f;
float LIMIT_DOF5_MIN = -95.0f, LIMIT_DOF5_MAX = 84.0f;
float LIMIT_GRIPPER_MIN = 0.0f, LIMIT_GRIPPER_MAX = 180.0f;
unsigned long last_limit_req_time = 0;

float real_q[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // {pitch, roll, dof3, dof4, dof5}
                                                  // P: commands tienen pitch y roll al revÃƒÂ©s:
                                                  // P:roll, pitch, dof3, dof4, dof5\n
bool new_q_angles = false;

const float HOME[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

float real_gripper = 0.0f;

float gripper_target = 0.0f;

float raw_angles[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
// {motorL, BevelL, motorR, BevelR, dof3, dof4, dof5}
bool new_raw_angles = false;

RobotDHParams params;

float target[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
// x, y, z in mm; roll, pitch, yaw in degrees: {x, y, z, roll, pitch, yaw}
bool new_target = false;

float ik_target[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float manual_q_seed[5]{};
uint32_t lastManualIkMessageMs = 0;

bool new_ik_target = false;

float target_step_lin = 2.5f; // 0.5f;
float target_step_ang = 1.0f; // 0.2f;

const float tolerance_mm = 0.5f;
const float tolerance_rad = 0.035f;
const int max_iterations = 250;

bool failed_ik = false;

// ===================================
// Nucleo Serial communication variables
// ===================================
const byte MAX_NUCLEO_BUFFER = 128;
char nucleoBuffer[MAX_NUCLEO_BUFFER];
byte nucleoBufIndex = 0;

const byte MAX_PC_BUFFER = 128;
char pcInputBuffer[MAX_PC_BUFFER];
bool pcInputExists = false;

// ===================================
// DEBUG variables
// ===================================
struct buttonInfo {
  uint16_t mask;
  const char *name;
};

// Tabla de referencia con tus botones estandarizados
const buttonInfo BUTTONS_TABLE[] = {
    {BTN_B, "BTN_B"},           {BTN_A, "BTN_A"},   {BTN_Y, "BTN_Y"},
    {BTN_X, "BTN_X"},           {BTN_L1, "BTN_L1"}, {BTN_R1, "BTN_R1"},
    {BTN_L2, "BTN_L2"},         {BTN_R2, "BTN_R2"}, {BTN_L3, "BTN_L3 (ThumbL)"},
    {BTN_R3, "BTN_R3 (ThumbR)"}};

float tempVal = 0.0f;

// ===================================
// DEBUG functions
// ===================================
void printPressedButtons(uint16_t positive_edges) {
  if (positive_edges == 0)
    return; // Si nadie tocÃƒÂ³ nada, salÃƒÂ­ rÃƒÂ¡pido

  char buffer[128];
  for (const auto &btn : BUTTONS_TABLE) {
    // Evaluamos si el bit de este botÃƒÂ³n en especÃƒÂ­fico estÃƒÂ¡ activo
    if (positive_edges & btn.mask) {
      snprintf(buffer, sizeof(buffer), "[DEBUG] Presionado: %s\n", btn.name);
      Serial.print(buffer); // O Serial.print(buffer); si estÃƒÂ¡s en micro
    }
  }
}

// ===================================
// Homography functions
// ===================================

// ===================================
// SERIAL monitor functions
// ===================================
void printRawAngles() {
  if (new_raw_angles) {
    new_raw_angles = false;
    Serial.println("================= SENSOR ANGLES =================");
    Serial.print("GDL1 (Left Bevel) -> Motor: ");
    Serial.print(raw_angles[0], 2);
    Serial.print("  |  Joint: ");
    Serial.println(raw_angles[1], 2);
    Serial.print("GDL2 (Right Bevel)-> Motor: ");
    Serial.print(raw_angles[2], 2);
    Serial.print("  |  Joint: ");
    Serial.println(raw_angles[3], 2);
    Serial.print("GDL3 (Elbow)     -> Joint: ");
    Serial.println(raw_angles[4], 2);
    Serial.print("GDL4 (Wrist 1)  -> Angle: ");
    Serial.println(raw_angles[5], 2);
    Serial.print("GDL5 (Wrist 2)   -> Target:");
    Serial.println(raw_angles[6], 2);
    Serial.println("=================================================");
  }
}

void printRealIkTargets() {
  if (new_q_angles) {
    robot_fkine(real_q, params, target);
  }
  if (new_ik_target || new_q_angles) {
    new_ik_target = false;
    new_q_angles = false;
    Serial.print("[IK TCP]   Pose -> X:");
    Serial.print(ik_target[0], 1);
    Serial.print(" Y:");
    Serial.print(ik_target[1], 1);
    Serial.print(" Z:");
    Serial.print(ik_target[2], 1);
    Serial.print(" Roll:");
    Serial.print(ik_target[3], 1);
    Serial.print(" Pitch:");
    Serial.print(ik_target[4], 1);
    Serial.print(" Yaw:");
    Serial.println(ik_target[5], 1);

    Serial.print("[REAL TCP] Pose -> X:");
    Serial.print(target[0], 1);
    Serial.print(" Y:");
    Serial.print(target[1], 1);
    Serial.print(" Z:");
    Serial.print(target[2], 1);
    Serial.print(" Roll:");
    Serial.print(target[3], 1);
    Serial.print(" Pitch:");
    Serial.print(target[4], 1);
    Serial.print(" Yaw:");
    Serial.println(target[5], 1);
  }
}

// ===================================
// UDP functions
// ===================================
// Controller UDP
uint8_t computeChecksum(const uint8_t *data, size_t length) {
  uint8_t xorVal = 0;
  for (size_t i = 0; i < length; i++) {
    xorVal ^= data[i];
  }
  return xorVal;
}

bool processUDPPacket(PS3ControlPacket &packetOut) {
  int packetSize = udpController.parsePacket();
  controllerLink.lastPacketSize = packetSize;
  if (packetSize <= 0) {
    if (packetSize < 0)
      ++controllerLink.socketErrors;
    return false;
  }
  ++controllerLink.received;

  if (packetSize != sizeof(PS3ControlPacket)) {
    ++controllerLink.badSize;
    return false;
  }

  // Capture the sender of this datagram, not a later socket operation.
  IPAddress sender = udpController.remoteIP();
  if (esp32IP_known && udpControllerStable && sender != esp32IP) {
    ++controllerLink.wrongPeer;
    return false;
  }

  PS3ControlPacket rxTemp{};
  if (udpController.read((uint8_t *)&rxTemp, sizeof(rxTemp)) != int(sizeof(rxTemp))) {
    ++controllerLink.badRead;
    return false;
  }

  uint8_t chk = computeChecksum((uint8_t *)&rxTemp, sizeof(PS3ControlPacket) - 1);
  if (chk != rxTemp.checksum) {
    ++controllerLink.badChecksum;
    return false;
  }

  if (udpControllerStable && (int32_t)(rxTemp.seq - lastSeqReceived) <= 0) {
    ++controllerLink.staleSequence;
    return false;
  }

  // Sequence gaps are diagnostic only. A reboot/peer change cannot be compared
  // reliably; uint32 subtraction supports normal sequence wrap.
  uint32_t sequenceStep = uint32_t(rxTemp.seq - lastSeqReceived);
  bool comparable = controllerLink.accepted && esp32IP_known && sender == esp32IP &&
                    (int32_t)sequenceStep > 0;
  if (comparable)
    controllerLink.missingSequences += sequenceStep - 1;

  // Register the validated controller peer before servicing the vision socket.
  // An initial camera event can arrive in the very same loop iteration.
  // A disconnected ESP32 may obtain a different DHCP address on reconnect.
  if (!esp32IP_known || sender != esp32IP) {
    cameraAlgorithm.invalidate();
    esp32IP = sender;
    esp32IP_known = true;
    Serial.print("[LINK] Controller peer: ");
    Serial.println(esp32IP.toString());
  }

  lastSeqReceived = rxTemp.seq;
  uint32_t now = millis();
  if (controllerLink.accepted) {
    controllerLink.lastAcceptedGapMs = uint32_t(now - lastTimeReceived);
    if (controllerLink.lastAcceptedGapMs > controllerLink.maxAcceptedGapMs)
      controllerLink.maxAcceptedGapMs = controllerLink.lastAcceptedGapMs;
  }
  if (controllerLink.awaitingRecovery) {
    controllerLink.recoveryGapMs = uint32_t(now - lastTimeReceived);
    controllerLink.recoverySequence = rxTemp.seq;
    controllerLink.recoverySequenceStep = comparable ? sequenceStep : 0;
    controllerLink.recoverySequenceComparable = comparable;
    controllerLink.recoveryPending = true;
    controllerLink.awaitingRecovery = false;
  }
  ++controllerLink.accepted;
  lastTimeReceived = now; // Watchdog refresh
  packetOut = rxTemp;

  return true;
}

void activate_UDP_Failsafe() {
  Serial.println(controllerLossRequiresStop()
                     ? "[WARN] Signal lost, failsafe activated"
                     : "[WARN] Controller signal lost; no motion-mode ESTOP required");
  char message[320];
  snprintf(message, sizeof(message),
           "[LINK] age=%lums pollGap=%lums maxPollGap=%lums maxVisionWork=%lums "
           "seq=%lu rx=%lu accepted=%lu missingSeq=%lu maxRxGap=%lums\n",
           (unsigned long)controllerLink.timeoutAgeMs,
           (unsigned long)controllerLink.timeoutPollGapMs,
           (unsigned long)controllerLink.maxPollGapMs,
           (unsigned long)controllerLink.maxVisionWorkMs,
           (unsigned long)controllerLink.timeoutSequence, (unsigned long)controllerLink.received,
           (unsigned long)controllerLink.accepted,
           (unsigned long)controllerLink.missingSequences,
           (unsigned long)controllerLink.maxAcceptedGapMs);
  Serial.print(message);
  snprintf(message, sizeof(message),
           "[LINK] rejected size=%lu read=%lu checksum=%lu peer=%lu sequence=%lu "
           "socketErrors=%lu visionTxFailures=%lu lastRxSize=%d\n",
           (unsigned long)controllerLink.badSize, (unsigned long)controllerLink.badRead,
           (unsigned long)controllerLink.badChecksum, (unsigned long)controllerLink.wrongPeer,
           (unsigned long)controllerLink.staleSequence, (unsigned long)controllerLink.socketErrors,
           (unsigned long)controllerLink.visionTxFailures, controllerLink.lastPacketSize);
  Serial.print(message);
}

void reportControllerRecovery() {
  if (!controllerLink.recoveryPending)
    return;
  controllerLink.recoveryPending = false;
  char message[224];
  snprintf(message, sizeof(message),
           "[LINK] Traffic resumed: gap=%lums seq=%lu seqStep=%lu comparable=%u "
           "missingSeq=%lu; %s\n",
           (unsigned long)controllerLink.recoveryGapMs,
           (unsigned long)controllerLink.recoverySequence,
           (unsigned long)controllerLink.recoverySequenceStep,
           unsigned(controllerLink.recoverySequenceComparable),
           (unsigned long)controllerLink.missingSequences,
           controllerLossRequiresStop() ? "emergency reset still required" : "non-motion operation continued");
  Serial.print(message);
}

void serviceControllerPackets() {
  uint32_t start = millis();
  if (controllerLink.polled) {
    controllerLink.lastPollGapMs = uint32_t(start - controllerLink.lastPollMs);
    if (controllerLink.lastPollGapMs > controllerLink.maxPollGapMs)
      controllerLink.maxPollGapMs = controllerLink.lastPollGapMs;
  }
  controllerLink.polled = true;
  controllerLink.lastPollMs = start;
  // Invalid/late datagrams must not hide a newer valid packet in the queue.
  // Bound the work while retaining short button presses and emergency edges.
  for (int n = 0; n < 8; ++n) {
    uint16_t previous = currentControlCommand.buttons;
    if (processUDPPacket(currentControlCommand)) {
      controllerPressedEdges |= currentControlCommand.buttons & ~previous;
      controllerEmergencyReceived |= (currentControlCommand.buttons & BTN_B) != 0;
      newControlCMD = true;
      udpControllerStable = true;
    }
    if (controllerLink.lastPacketSize <= 0 || uint32_t(millis() - start) >= 2)
      break;
  }
  if (udpControllerStable && uint32_t(millis() - lastTimeReceived) > TIMEOUT_FAILSAFE_MS) {
    // A BT/controller gap does not establish that the independent camera task
    // reset. Preserve pending/confirmed algorithms in non-motion states.
    // Camera online/offline events, changed peers and transaction faults still
    // invalidate the cache through their existing paths.
    if (controllerLossRequiresStop()) cameraAlgorithm.invalidate();
    udpControllerStable = false;
    controllerLink.awaitingRecovery = true;
    if (!controllerTimedOutThisLoop) {
      controllerLink.timeoutAgeMs = uint32_t(millis() - lastTimeReceived);
      controllerLink.timeoutPollGapMs = controllerLink.lastPollGapMs;
      controllerLink.timeoutSequence = lastSeqReceived;
    }
    controllerTimedOutThisLoop = true;
  }
}

// Vision UDP
void sendVisionPacket(const VisionPacket &pkt) {
  if (!cameraUDP.sendTo(esp32IP, cameraUDPport, (const uint8_t *)&pkt, sizeof(pkt)))
    ++controllerLink.visionTxFailures;
}

void sendCriticalMessage(typeMsg type, uint8_t param = 0) {
  if (pendingESPmsg.pending || !esp32IP_known) {
    cameraAlgorithm.invalidate();
    communicationError = true;
    return;
  }
  pendingESPmsg.pkt = VisionPacket{};
  pendingESPmsg.pkt.type = type;
  pendingESPmsg.pkt.msgID = globalMsgCounter++;
  if (!globalMsgCounter)
    globalMsgCounter = 1;
  if (type == CMD_SWITCH_ALGO) {
    cameraAlgorithm.begin(param);
    pendingESPmsg.pkt.config.ALGO_ID = param;
  }
  messageAck = false;
  lastAckType = 0;
  pendingESPmsg.pending = true;
  pendingESPmsg.triesLeft = MAX_TRIES;
  pendingESPmsg.lastPing = millis();
  sendVisionPacket(pendingESPmsg.pkt);
}
void managePingsVision() {
  if (!pendingESPmsg.pending)
    return;
  if (millis() - pendingESPmsg.lastPing < RETRY_TIMEOUT_MS)
    return;
  if (!pendingESPmsg.triesLeft) {
    cameraAlgorithm.invalidate();
    pendingESPmsg.pending = false;
    communicationError = true;
    return;
  }
  --pendingESPmsg.triesLeft;
  pendingESPmsg.lastPing = millis();
  sendVisionPacket(pendingESPmsg.pkt);
}
void manageVisionPort() {
  for (int n = 0; n < 8; ++n) {
    int size = cameraUDP.parsePacket();
    if (size <= 0)
      break;
    if (!esp32IP_known || cameraUDP.remoteIP() != esp32IP || size != sizeof(VisionPacket)) {
      // parsePacket() already consumes the datagram; skip it with the next parse.
      continue;
    }
    VisionPacket p{};
    if (cameraUDP.read((uint8_t *)&p, sizeof(p)) != sizeof(p))
      continue;
    if (p.type == ACK_OK) {
      if (pendingESPmsg.pending && p.msgID == pendingESPmsg.pkt.msgID &&
          p.ack.typeAck == pendingESPmsg.pkt.type) {
        lastAckType = p.ack.typeAck;
        pendingESPmsg.pending = false;
        messageAck = true;
        if (p.ack.typeAck == CMD_SWITCH_ALGO)
          cameraAlgorithm.acknowledge();
      }
      continue;
    }
    if (p.type == DATA_OBJECTS) {
      if (p.vision.amountoObjects > 10)
        continue;
      totalObjects = p.vision.amountoObjects;
      memcpy(latestReceived, p.vision.objList, totalObjects * sizeof(objectVision));
      visionTimestampMs = millis();
      newLatest = true;
      continue;
    }
    if (p.type != EVT_CAM_ONLINE && p.type != EVT_CAM_OFFLINE && p.type != EVT_SWITCHED_ALGO &&
        p.type != EVT_NO_ALGO)
      continue;
    VisionPacket ack{};
    ack.type = ACK_OK;
    ack.msgID = p.msgID;
    ack.ack.typeAck = p.type;
    sendVisionPacket(ack);
    if (p.type == EVT_CAM_ONLINE) {
      cameraAlgorithm.invalidate();
      cameraOffline = false;
    }
    if (p.type == EVT_CAM_OFFLINE) {
      cameraAlgorithm.invalidate();
      cameraOffline = true;
    }
    if (p.type == EVT_SWITCHED_ALGO) {
      cameraAlgorithm.switched();
      algoSwitched = true;
    }
    if (p.type == EVT_NO_ALGO) {
      cameraAlgorithm.invalidate();
      invalidAlgo = true;
    }
  }
}
void processNucleoCMD() {
  if (nucleoBuffer[0] == '\0' || nucleoBuffer[1] != ':')
    return;
  float a[12];
  char *d = nucleoBuffer + 2;
  int n = 0;
  switch (nucleoBuffer[0]) {
  case 'D':
    jointMotionTest.receive(nucleoBuffer, millis());
    break;
  case 'J':
  case 'O':
  case 'H':
    jointWizard.receive(nucleoBuffer, millis());
    break;
  case 'A':
    n = sscanf(d, "%f,%f,%f,%f,%f,%f", &a[0], &a[1], &a[2], &a[3], &a[4], &a[5]);
    if (n != 6)
      return;
    for (int i = 0; i < 6; ++i)
      if (!isfinite(a[i]))
        return;
    for (int i = 0; i < 5; ++i) {
      if (fabsf(real_q[i] - a[i]) > .01f)
        new_q_angles = true;
      real_q[i] = armFeedback.q[i] = a[i];
    }
    real_gripper = a[5];
    armFeedback.timestampMs = millis();
    ++armFeedback.sequence;
    armFeedback.valid = true;
    break;
  case 'R':
    n = sscanf(d, "%f,%f,%f,%f,%f,%f,%f", &a[0], &a[1], &a[2], &a[3], &a[4], &a[5], &a[6]);
    if (n != 7)
      return;
    for (int i = 0; i < 7; ++i)
      if (!isfinite(a[i]))
        return;
    for (int i = 0; i < 7; ++i) {
      if (fabsf(raw_angles[i] - a[i]) > .01f)
        new_raw_angles = true;
      raw_angles[i] = a[i];
    }
    break;
  case 'L':
    n = sscanf(d, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", &a[0], &a[1], &a[2], &a[3], &a[4], &a[5],
               &a[6], &a[7], &a[8], &a[9], &a[10], &a[11]);
    if (n != 12)
      return;
    for (int i = 0; i < 12; ++i)
      if (!isfinite(a[i]))
        return;
    for (int i = 0; i < 12; i += 2)
      if (a[i] >= a[i + 1])
        return;
    LIMIT_ROLL_MIN = a[0];
    LIMIT_ROLL_MAX = a[1];
    LIMIT_PITCH_MIN = a[2];
    LIMIT_PITCH_MAX = a[3];
    LIMIT_DOF3_MIN = a[4];
    LIMIT_DOF3_MAX = a[5];
    LIMIT_DOF4_MIN = a[6];
    LIMIT_DOF4_MAX = a[7];
    LIMIT_DOF5_MIN = a[8];
    LIMIT_DOF5_MAX = a[9];
    LIMIT_GRIPPER_MIN = a[10];
    LIMIT_GRIPPER_MAX = a[11];
    updateLimits(params, a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8], a[9]);
    limits_received = true;
    Serial.println("[SYSTEM] Limits saved");
    break;
  default:
    break;
  }
}
void readNucleoSerial() {
  int budget = 256;
  while (budget-- && Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      if (!discardNucleoLine) {
        nucleoBuffer[nucleoBufIndex] = '\0';
        processNucleoCMD();
      }
      nucleoBufIndex = 0;
      discardNucleoLine = false;
    } else if (c != '\r' && !discardNucleoLine) {
      if (nucleoBufIndex < MAX_NUCLEO_BUFFER - 1)
        nucleoBuffer[nucleoBufIndex++] = c;
      else {
        discardNucleoLine = true;
        nucleoBufIndex = 0;
      }
    }
  }
}

inline int8_t pocketSign(int8_t val) {
  return (val > joystick_deadzone) - (val < -joystick_deadzone);
}

// ===================================
// FSM DEFINITION
// ===================================
enum State {
  WAITING_CONNECTION,
  MODE_SELECTOR,
  EMERGENCY_STOP,
  MANUAL_0,
  MANUAL_1,
  AUTO_MENU,
  AUTO_MODE,
  CALIBRATION_MENU,
  TRAJECTORY_TEST,
  WAITING_ALGO_SWITCH,
  CAMERA_CALIBRATION,
  VELOCITY_CALIBRATION,
  JOINT_CALIBRATION,
  JOINT_MOTION_TEST,
  TUNING_MENU,
  AUTO_TUNING
};

State currentState = WAITING_CONNECTION;
State lastState = EMERGENCY_STOP;
State tuningReturnState = CALIBRATION_MENU;

// Keep Arduino's generated prototype from referencing State before its enum.
bool usesCamera(State s);

uint8_t menu_option0 = 0;
uint8_t menu_option1 = 0;
uint8_t menu_option2 = 0;
// Calibration navigation must not change AUTO's ID-classification option.
uint8_t calibrationMenuOption = 0;

const char *const opts_CALIBRATION_MENU[6] = {"CAMERA CALIBRATION", "BELT VELOCITY ESTIMATION",
    "TRAJECTORY TEST", "JOINT MOTION TEST", "TUNING & STORAGE", "AUTO PICKUP TUNING"};

bool autoMovesArm() {return currentState==AUTO_TUNING || menu_option2==0;}
bool autoRespectsId() {return currentState!=AUTO_TUNING && menu_option0==1;}

const char *const opts_AUTO_MENU_ID_TRACKING[2] = {"ID IGNORANT", "ID CLASIFICATION"};

const char *const opts_AUTO_MENU_IK[2] = {"MOVE ARM", "ONLY VISION"};

// ===================================
// SETUP
// ===================================

bool takeAck(typeMsg t) {
  if (!messageAck || lastAckType != t)
    return false;
  messageAck = false;
  return true;
}
bool feedbackFresh() {
  return armFeedback.valid && millis() - armFeedback.timestampMs <= autoConfig.feedbackTimeoutMs;
}
bool validAutoJoints(const float *q) {
  if (!feedbackFresh() || !limits_received)
    return false;
  for (int i = 0; i < 5; ++i) {
    const float radians = q[i] * PI / 180;
    if (!isfinite(q[i]) || radians < params.q_min[i] || radians > params.q_max[i])
      return false;
  }
  return true;
}
bool sendAutoJoints(const float *q) {
  if (!validAutoJoints(q))
    return false;
  char b[96];
  int n = snprintf(b, sizeof(b), "P:%.2f,%.2f,%.2f,%.2f,%.2f\n", q[1], q[0], q[2], q[3], q[4]);
  return n > 0 && n < int(sizeof(b)) && Serial1.write((const uint8_t *)b, n) == size_t(n);
}
bool sendAutoGripper(float angle) {
  if (!feedbackFresh() || !isfinite(angle) || angle < LIMIT_GRIPPER_MIN ||
      angle > LIMIT_GRIPPER_MAX)
    return false;
  char b[32];
  int n = snprintf(b, sizeof(b), "G:%.2f\n", angle);
  return n > 0 && n < int(sizeof(b)) && Serial1.write((const uint8_t *)b, n) == size_t(n);
}
bool sendAutoHome(const float *q, float angle) {
  if (!validAutoJoints(q) || !isfinite(angle) || angle < LIMIT_GRIPPER_MIN ||
      angle > LIMIT_GRIPPER_MAX)
    return false;
  // Keep mode, HOME and open in order in one short write. Current Nucleo V2
  // uses a circular-DMA command FIFO; batching also supports its older receiver.
  char b[128];
  int n = snprintf(b, sizeof(b), "S:AUTO\nP:%.2f,%.2f,%.2f,%.2f,%.2f\nG:%.2f\n", q[1], q[0], q[2],
                   q[3], q[4], angle);
  if (n <= 0 || n >= int(sizeof(b)) || Serial1.write((const uint8_t *)b, n) != size_t(n))
    return false;
  Serial.print("[ARM TX] HOME batch: ");
  Serial.print(b);
  return true;
}
void printArmProgress(uint32_t now, bool force) {
  ClassificationPlanReport classification;
  if(arm.takeClassificationPlan(classification))printClassificationPlan(Serial,classification);
  ArmDiagnosticReport diagnostic;
  while (arm.takeDiagnostic(diagnostic))
    printArmDiagnostic(Serial, diagnostic);
  if(arm.takeProfileReport()) {
    const AutoConfig &c=arm.configuration(); char b[256];
    snprintf(b,sizeof(b),"[PICK PROFILE] %s track=%lu margin=%lums descent=%lums closeLead=%lums closeDuration=%lums openDuration=%lums approachZ=%.2f pickupZ=%.2f\n",
      arm.activeProfile() ? "TALL (height>width)":"WIDE (height<=width)",
      (unsigned long)arm.trackId(),(unsigned long)c.pickUpMarginMs,(unsigned long)c.descentLeadMs,
      (unsigned long)c.closingLeadMs,(unsigned long)c.closingDurationMs,
      (unsigned long)c.openingDurationMs,c.approachZ,c.pickupZ);
    Serial.print(b);
  }
  if (arm.takeOrientationReport()) {
    char message[128];
    snprintf(message, sizeof(message), "[PICK] Orientation locked track=%lu yaw=%.0fdeg at descent\n",
             (unsigned long)arm.trackId(), arm.selectedPickupYaw());
    Serial.print(message);
  }
  if (!force && (!arm.busy() || uint32_t(now - lastArmProgressMs) < 1000))
    return;
  lastArmProgressMs = now;
  ArmProgress p = arm.progress(armFeedback, now);
  const char *stage = arm.busy() ? arm.phaseName() : arm.faultStage();
  char b[384];
  snprintf(b, sizeof(b),
           "[ARM] %s elapsed=%lums feedbackAge=%lums valid=%u inTolerance=%u stable=%lums "
           "TCPerror=%.2fmm entry=%u TCPentry/hold=%.2f/%.2fmm "
           "tickGap=%lums maxTickGap=%lums feedbackInterval=%lums\n",
           stage, (unsigned long)p.elapsedMs, (unsigned long)p.feedbackAgeMs, p.feedbackValid,
           p.withinTarget, (unsigned long)p.stableMs, p.positionErrorMm, p.withinEntry,
           p.positionEntryToleranceMm, p.positionHoldToleranceMm,
           (unsigned long)p.tickGapMs, (unsigned long)p.maximumTickGapMs,
           (unsigned long)p.feedbackIntervalMs);
  Serial.print(b);
  snprintf(b, sizeof(b),
           "[ARM] target[pitch,roll,j3,j4,j5]=%.2f,%.2f,%.2f,%.2f,%.2f "
           "error[actual-target]=%.2f,%.2f,%.2f,%.2f,%.2fdeg "
           "jointTol=%.2f,%.2f,%.2f,%.2f,%.2fdeg q4entry/hold=%.2f/%.2fdeg holdActive=%u\n",
           p.target[0], p.target[1], p.target[2], p.target[3], p.target[4], p.error[0], p.error[1],
           p.error[2], p.error[3], p.error[4], p.jointToleranceDeg[0], p.jointToleranceDeg[1],
           p.jointToleranceDeg[2], p.jointToleranceDeg[3], p.jointToleranceDeg[4],
           p.jointEntryToleranceDeg[3], p.jointHoldToleranceDeg[3], p.jointHoldActive);
  Serial.print(b);
}
void printIkDiagnostic(const char *label, const float pose[6], const float seed[5],
                       const IkReport &report) {
  char b[320];
  snprintf(b, sizeof(b),
           "[IK %s] task=%s reason=%s iterations=%d posError=%.3fmm "
           "controlledRotError=%.3fdeg fullRotError=%.3fdeg\n",
           label, ik_task_name(report.task), ik_failure_name(report.failure), report.iterations,
           report.positionErrorMm, report.controlledRotationErrorRad * 180 / PI,
           report.fullRotationErrorRad * 180 / PI);
  Serial.print(b);
  snprintf(b, sizeof(b),
           "[IK %s] TCP request=%.2f,%.2f,%.2f,%.2f,%.2f,%.2f "
           "seed[pitch,roll,j3,j4,j5]=%.2f,%.2f,%.2f,%.2f,%.2f\n",
           label, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], seed[0], seed[1], seed[2],
           seed[3], seed[4]);
  Serial.print(b);
}
void printPlanRejection() {
  Serial.print("[PLAN] Rejected: ");
  Serial.println(arm.rejectionReason());
  ArmPlanReport p = arm.planReport();
  printIkDiagnostic(arm.planTargetName(), p.pose, p.seed, p.ik);
  char b[160];
  snprintf(b, sizeof(b), "[PLAN] candidate[pitch,roll,j3,j4,j5]=%.2f,%.2f,%.2f,%.2f,%.2f\n",
           p.candidate[0], p.candidate[1], p.candidate[2], p.candidate[3], p.candidate[4]);
  Serial.print(b);
}
bool validPose(const float *p) {
  for (int i = 0; i < 6; ++i)
    if (!isfinite(p[i]))
      return false;
  return fabsf(p[4]) < 89;
}
// One source of truth for both the interlock and its operator-facing reason.
// Return the first blocker; fixing it may expose another prerequisite.
const char *armConfigurationIssue(const AutoConfig &c, bool requireClassDestinations) {
  if (!limits_received)
    return "Joint/gripper limits not received from Nucleo";
  if (!feedbackFresh())
    return "Nucleo feedback invalid or stale";
  if (!isfinite(c.pickupZ) || !isfinite(c.approachZ) || !isfinite(c.pickupRoll) ||
      !isfinite(c.pickupPitch) || !isfinite(c.pickupYaw) || fabsf(c.pickupPitch) >= 89 ||
      !isfinite(c.approachRoll) || !isfinite(c.approachPitch) || !isfinite(c.approachYaw))
    return "Pickup/approach pose invalid";
  if (!validArrivalConfiguration(c))
    return "Arrival tolerances invalid";
  if (!(c.maximumCaptureErrorYmm > 0))
    return "maximumCaptureErrorYmm must be positive";
  if (!validPickupTiming(c))
    return "Invalid pickup timing: require descentLead>=closingLead, duration>=closingLead and positive margin/timers";
  if (c.approachZ <= c.pickupZ)
    return "approachZ must be above pickupZ";
  if (!validPose(c.defaultDestination))
    return "Default classification pose invalid";
  if (!isfinite(c.openAngle) || c.openAngle < LIMIT_GRIPPER_MIN ||
      c.openAngle > LIMIT_GRIPPER_MAX)
    return "openAngle outside Nucleo gripper limits";
  if (!isfinite(c.closeAngle) || c.closeAngle < LIMIT_GRIPPER_MIN ||
      c.closeAngle > LIMIT_GRIPPER_MAX)
    return "closeAngle outside Nucleo gripper limits";
  // Every run and cancel-return uses joint HOME = {0,0,0,0,0}.
  for (int i = 0; i < 5; ++i)
    if (params.q_min[i] > 0 || params.q_max[i] < 0)
      return "Joint limits exclude HOME (zero joint angles)";
  if (requireClassDestinations) {
    if (!Classification::mappingValid())
      return "Classification part-ID mapping invalid";
    bool enabled = false;
    for (const Destination &d : c.destinations)
      if (d.enabled) {
        if (!validPose(d.pose))
          return "Enabled classification pose invalid";
        enabled = true;
      }
    if (!enabled)
      return "No classification destinations enabled";
    if (!Classification::activeRouteCount(c))
      return "No part IDs mapped to enabled classification destinations";
  }
  return nullptr;
}
bool armConfigurationReady(const AutoConfig &c, bool requireClassDestinations) {
  return armConfigurationIssue(c, requireClassDestinations) == nullptr;
}
const char *autoReadinessIssue() {
  if (cameraOffline)
    return "Camera unavailable";
  if (!homographyDone || !camera_calib.valid)
    return "Camera calibration missing or invalid";
  if (!velocityCalibrated || !mainTracker.calibrated())
    return "Belt velocity calibration missing or invalid";
  if (!autoMovesArm())
    return nullptr;
  if (jointWizard.statusReceived && jointWizard.source == JC_STORAGE_CORRUPT)
    return "Nucleo joint calibration storage corrupt; use joint calibration recovery";
  if(const char *issue=activeTuning.issue())return issue;
  for(unsigned i=0;i<2;++i) {
    AutoConfig c=autoConfig;activeTuning.apply(c,i);
    if(const char *issue=armConfigurationIssue(c,autoRespectsId()))return issue;
  }
  return nullptr;
}
bool autoReady() {
  return autoReadinessIssue() == nullptr;
}
void printReadinessFailure(const char *mode, const char *reason, const AutoConfig &c) {
  char b[240];
  snprintf(b, sizeof(b), "[%s] Cannot start: %s\n", mode, reason);
  Serial.print(b);
  snprintf(b, sizeof(b),
           "[%s] gripper openAngle=%.2f closeAngle=%.2f allowed=%.2f..%.2fdeg; "
           "limitsReceived=%u feedbackValid=%u feedbackAge=%lums\n",
           mode, c.openAngle, c.closeAngle, LIMIT_GRIPPER_MIN, LIMIT_GRIPPER_MAX,
           unsigned(limits_received), unsigned(armFeedback.valid),
           (unsigned long)(millis() - armFeedback.timestampMs));
  Serial.print(b);
}
void latchEmergency(const char *reason) {
  tuningMenuRequested=false;
  autoTuningLaunchPending=pickupSaveRequested=false;
  jointWizard.abandon();
  Serial1.print("S:ESTOP\n");
  jointMotionTest.emergency(millis());
  Serial.print("[ESTOP] ");
  Serial.println(reason);
  entered_ES = true;
  leave_ES = false;
  holdingResetCombo = false;
  arm.cancel();
  mainTracker.clear();
  newLatest = false;
  streamStarted = false;
  stopRequested = true;
  pendingESPmsg.pending = false;
  messageAck = false;
  currentState = EMERGENCY_STOP;
}
bool usesCamera(State s) {
  // The menu and dry run must remain accessible with no camera/calibrations.
  return s == AUTO_MENU || s == AUTO_MODE || s == AUTO_TUNING || s == WAITING_ALGO_SWITCH || s == CAMERA_CALIBRATION ||
         s == VELOCITY_CALIBRATION;
}
bool controllerLossRequiresStop() {
  // Enforce throughout motion-capable modes, including idle/holding periods.
  // Menus, camera calibrations, algorithm switching and ONLY VISION do not move.
  // A latched emergency must also retain its reset-required diagnostics.
  return currentState == EMERGENCY_STOP || currentState == MANUAL_0 || currentState == MANUAL_1 ||
         currentState == TRAJECTORY_TEST || currentState == JOINT_MOTION_TEST ||
         currentState == JOINT_CALIBRATION || currentState == AUTO_TUNING ||
         (currentState == AUTO_MODE && menu_option2 == 0);
}
void serviceImmediateControllerFaults() {
  if (currentState == WAITING_CONNECTION || currentState == EMERGENCY_STOP) return;
  if (controllerTimedOutThisLoop && controllerLossRequiresStop())
    latchEmergency("Controller connection lost");
  else if (controllerEmergencyReceived)
    latchEmergency("Emergency button");
}
void serviceGlobalFaults() {
  if (currentState == WAITING_CONNECTION || currentState == EMERGENCY_STOP)
    return;
  if (controllerLossRequiresStop() && (!udpControllerStable || controllerTimedOutThisLoop)) {
    latchEmergency("Controller connection lost");
    return;
  }
  if ((currentControlCMD.buttons & BTN_B) || controllerEmergencyReceived) {
    latchEmergency("Emergency button");
    return;
  }
  if (usesCamera(currentState) && (cameraOffline || communicationError)) {
    latchEmergency(cameraOffline ? "Camera offline" : "ESP32 transaction failed");
    return;
  }
  if ((currentState == TRAJECTORY_TEST || currentState == JOINT_MOTION_TEST ||
       currentState == AUTO_TUNING || (currentState == AUTO_MODE && menu_option2 == 0)) &&
      arm.busy() && !feedbackFresh())
    latchEmergency("Nucleo feedback stale");
}
void printAutoMenu() {
  Serial.print(opts_AUTO_MENU_ID_TRACKING[menu_option0]);
  char message[32];
  snprintf(message, sizeof(message), " | MODEL ID %u | ",
           unsigned(menu_option1 == 0 ? CUSTOM_MODEL1_ID : CUSTOM_MODEL2_ID));
  Serial.print(message);
  Serial.println(opts_AUTO_MENU_IK[menu_option2]);
  Serial.println("[BELT] Fixed AprilTag calibration; speed changes are NOT detected. "
                 "Operator must recalibrate if camera placement or actual belt speed changes.");
}
void beginStreamEnd(AutoRun next) {
  sendCriticalMessage(CMD_END_STREAM);
  autoRun = next;
}
void selectNextPart(uint32_t now) {
  static uint32_t lastClassWaitLogMs = 0;
  if (stopRequested || !autoMovesArm() || autoRun != AutoRun::Running || arm.busy())
    return;
  Part p;
  AutoPick::Request best, request;
  bool selected = false;
  for (int i = 0; i < MAX_PARTS; ++i)
    if (mainTracker.candidate(i, p, now)) {
      AutoPick::Admission admission = AutoPick::evaluate(p, mainTracker.velocity(), autoConfig,
                                                        autoRespectsId(), now, request, &activeTuning);
      if (admission == AutoPick::Admission::AwaitingClassification) {
        if (uint32_t(now - lastClassWaitLogMs) >= 500) {
          char message[192];
          snprintf(message, sizeof(message),
                   "[AUTO] Waiting for classification track=%lu pendingID=%u samples=%u/%u span=%lu/%lums\n",
                   (unsigned long)p.trackId, unsigned(p.pendingClass), unsigned(p.classVotes),
                   unsigned(CameraTracking::ClassificationConfidence::minimumSamples),
                   (unsigned long)uint32_t(p.classLastMs - p.classSinceMs),
                   (unsigned long)CameraTracking::ClassificationConfidence::minimumStableMs);
          Serial.print(message);
          lastClassWaitLogMs = now;
        }
        continue;
      }
      if (admission == AutoPick::Admission::IgnoredId ||
          admission == AutoPick::Admission::OutsidePickupRange) {
        mainTracker.retire(p.trackId);
        ++ignoredParts;
        if (admission == AutoPick::Admission::OutsidePickupRange)
          ++outsidePickupParts;
        continue;
      }
      if (admission == AutoPick::Admission::TooLate) {
        char message[160];
        snprintf(message, sizeof(message),
                 "[AUTO] Missed track=%lu: preparation deadline reached; classification=%s\n",
                 (unsigned long)p.trackId,
                 partClassificationStable(p) ? "stable" : "pending");
        Serial.print(message);
        mainTracker.retire(p.trackId);
        ++auto_parts_missed;
        continue;
      }
      if (admission != AutoPick::Admission::Eligible) {
        mainTracker.retire(p.trackId);
        ++rejectedParts;
        continue;
      }
      if (!selected || request.arrivalDelayMs < best.arrivalDelayMs) {
        best = request;
        selected = true;
      }
    }
  if (selected && mainTracker.reserve(best.part.trackId)) {
    arm.start(best.part, mainTracker.velocity(), best.destination, best.config, armFeedback, now,
              &best.profiles);
    char message[288];
    snprintf(message, sizeof(message),
             "[AUTO] Assigned track=%lu part ID=%u bin=%u X=%.2f Y=%.2f "
             "pickupZ=%.2f vy=%.2fmm/s closeIn=%.0fms profile=%s idExtraX=%+.2fmm\n",
             (unsigned long)best.part.trackId, unsigned(best.part.classId), unsigned(best.binId),
             best.part.x, best.part.y, best.config.pickupZ, mainTracker.velocity(), best.closingDelayMs,
             best.part.boxHeight>best.part.boxWidth ? "TALL":"WIDE",best.idExtraXmm);
    Serial.print(message);
    if (autoRespectsId()) {
      snprintf(message, sizeof(message),
               "[AUTO] Classification locked track=%lu ID=%u bin=%u samples=%u span=%lums\n",
               (unsigned long)best.part.trackId, unsigned(best.part.classId), unsigned(best.binId),
               unsigned(best.part.classVotes),
               (unsigned long)uint32_t(best.part.classLastMs - best.part.classSinceMs));
      Serial.print(message);
    }
  }
}
void serviceAlgorithmSelection() {
  uint8_t algo = (requestedTagsAlgo || requestedAlgoVelocity) ? APRIL_TAG_ALGO_ID
                 : (autoTuningLaunchPending || menu_option1 == 0 ? CUSTOM_MODEL1_ID : CUSTOM_MODEL2_ID);
  if (currentState != lastState) {
    algoSwitched = invalidAlgo = switchAcked = false;
    newLatest = false;
    lastState = currentState;
    if (cameraAlgorithm.matches(algo) && !cameraOffline && !communicationError &&
        !pendingESPmsg.pending) {
      char message[96];
      snprintf(message, sizeof(message), "[CAM] Algorithm %u already confirmed; reusing\n", unsigned(algo));
      Serial.print(message);
      currentState = requestedTagsAlgo ? CAMERA_CALIBRATION
                     : requestedAlgoVelocity ? VELOCITY_CALIBRATION
                     : autoTuningLaunchPending ? AUTO_TUNING : AUTO_MODE;
      return;
    }
    sendCriticalMessage(CMD_SWITCH_ALGO, algo);
    taskStartedMs = millis();
  }
  if (takeAck(CMD_SWITCH_ALGO)) switchAcked = true;
  if (invalidAlgo) {
    cameraAlgorithm.invalidate();
    latchEmergency("Camera algorithm unavailable");
    return;
  }
  if (uint32_t(millis() - taskStartedMs) > 12000) {
    cameraAlgorithm.invalidate();
    latchEmergency("Algorithm switch timeout");
    return;
  }
  if (switchAcked && algoSwitched && cameraAlgorithm.matches(algo)) {
    algoSwitched = false;
    newLatest = false;
    currentState = requestedTagsAlgo ? CAMERA_CALIBRATION
                   : requestedAlgoVelocity ? VELOCITY_CALIBRATION
                   : autoTuningLaunchPending ? AUTO_TUNING : AUTO_MODE;
  }
}
void finishCalibrationStream() {
  sendCriticalMessage(CMD_END_STREAM);
  calPhase = CalPhase::Ending;
}
void printVelocityCalibrationReport(const char *label) {
  BeltCalibrationReport r = beltCalibration.report();
  char message[384];
  snprintf(message, sizeof(message),
           "[CAL %s] reason=%s samples=%u span=%lums travel(-Y)=%.2fmm deltaX=%.2fmm "
           "vy=%.2fmm/s RMS=%.2fmm pixel={%u,%u} world={%.2f,%.2f}mm outsideROI=%lu "
           "ignoredIDs=%lu repeats=%lu tagLocked=%u tagID=%u\n",
           label, belt_calibration_issue_name(r.issue), r.samples, (unsigned long)r.spanMs,
           r.travelMm, r.deltaXmm, r.velocityMmS, r.rmsMm, unsigned(r.pixelX),
           unsigned(r.pixelY), r.worldX, r.worldY, (unsigned long)r.outsideRegion,
           (unsigned long)r.ignoredIds, (unsigned long)r.repeatedPositions,
           unsigned(r.tagLocked), unsigned(r.tagId));
  Serial.print(message);
}

bool velocityLaneOccupied() {
  // Clearing is stricter than sampling: even a partly overlapping tag blocks
  // rearming. Corner tags must be completely outside the velocity rectangle.
  if (totalObjects > 10) return true;
  const auto &r = CameraTracking::VelocityRegion::rectangle;
  for (unsigned i = 0; i < totalObjects; ++i) {
    const auto &d = latestReceived[i];
    if (unsigned(d.ID) - unsigned(CameraTracking::VelocityRegion::minimumTagId) >
        unsigned(CameraTracking::VelocityRegion::maximumTagId) -
            unsigned(CameraTracking::VelocityRegion::minimumTagId)) continue;
    if (2 * int32_t(d.xCenter) + d.width >= 2 * r.left &&
        2 * int32_t(d.xCenter) - d.width <= 2 * r.right &&
        2 * int32_t(d.yCenter) + d.height >= 2 * r.top &&
        2 * int32_t(d.yCenter) - d.height <= 2 * r.bottom) return true;
  }
  return false;
}

void serviceVelocityCalibration(uint32_t now, uint16_t pressed) {
    if (currentState != lastState) {
      velocityCalibrated = false;
      mainTracker.setVelocity(0);
      beltCalibration.reset();
      beltCalibrationSession = BeltCalibrationSession{};
      lastVelocityCalibrationLogMs = now;
      newLatest = false;
      sendCriticalMessage(CMD_START_STREAM);
      calPhase = CalPhase::Starting;
      lastState = currentState;
      Serial.println("[CAL] Use ONE flat moving AprilTag ID 0..3 inside the ROI; corner tags stay outside");
      Serial.println("[BELT] Operator must recalibrate whenever actual belt speed changes; AUTO does not measure belt speed");
      Serial.println("[CAL] Repeated tag centres count once; repeats do not extend the observation span");
      char message[192];
      snprintf(message, sizeof(message),
               "[CAL] Collecting %u accepted passes; rejected passes are retried. L1: cancel\n",
               BeltCalibrationSessionConfig::requiredRuns);
      Serial.print(message);
      Serial.println("[CAL] Keep the moving tag OUT of the ROI; wait for the PLACE TAG prompt");
      snprintf(message, sizeof(message),
               "[CAL] Requires %u samples over >=%lums, >=%.0fmm along -Y, RMS<=%.1fmm; "
               "sample interval >=%lums\n",
               BeltCalibrationLimits::minimumSamples,
               (unsigned long)BeltCalibrationLimits::minimumSpanMs,
               BeltCalibrationLimits::minimumTravelMm, BeltCalibrationLimits::maximumRmsMm,
               (unsigned long)BeltCalibrationLimits::sampleIntervalMs);
      Serial.print(message);
      const auto &region = CameraTracking::VelocityRegion::rectangle;
      snprintf(message, sizeof(message),
               "[CAL] Whole box must fit in pixel ROI X=%ld..%ld Y=%ld..%ld; "
               "edit CameraTrackingConfig.h\n", (long)region.left, (long)region.right,
               (long)region.top, (long)region.bottom);
      Serial.print(message);
    }
    if (calPhase == CalPhase::Starting && takeAck(CMD_START_STREAM)) {
      calPhase = CalPhase::Collecting;
      taskStartedMs = now;
      newLatest = false;
    }
    if ((pressed & BTN_L1) && calPhase != CalPhase::Ending &&
        !beltCalibrationSession.cancelRequested) {
      beltCalibrationSession.cancelRequested = true;
      Serial.println("[CAL] Cancelled; partial calibration discarded; belt velocity unavailable");
    }
    // Finish a pending START handshake before END, so a late START cannot
    // restart the stream after cancellation.
    if (beltCalibrationSession.cancelRequested && calPhase == CalPhase::Collecting) {
      finishCalibrationStream();
    }
    if (calPhase == CalPhase::Collecting) {
      if (newLatest) {
        newLatest = false;
        if (beltCalibrationSession.phase == BeltCalibrationSession::Phase::Clearing) {
          if (beltCalibrationSession.observeClear(velocityLaneOccupied(), now)) {
            beltCalibration.reset();
            lastVelocityCalibrationLogMs = now;
            char message[160];
            snprintf(message, sizeof(message),
                     "[CAL] PLACE TAG: run %u/%u. Put ONE flat tag on the belt now\n",
                     beltCalibrationSession.accepted + 1, BeltCalibrationSessionConfig::requiredRuns);
            Serial.print(message);
          }
          return; // The clear frame is never a measurement for the next pass.
        }
        unsigned before = beltCalibration.samples();
        beltCalibration.addFrame(latestReceived, totalObjects, visionTimestampMs, camera_calib);
        if (!before && beltCalibration.seen()) {
          beltCalibrationSession.phase = BeltCalibrationSession::Phase::Recording;
          taskStartedMs = now;
          Serial.println("[CAL] Recording pass; let the tag travel through and leave the ROI");
        }
        if ((before || beltCalibration.seen() || beltCalibration.failed()) &&
            ((!before && beltCalibration.samples()) || now - lastVelocityCalibrationLogMs >= 250)) {
          printVelocityCalibrationReport("SAMPLE");
          lastVelocityCalibrationLogMs = now;
        }
      }
      if (beltCalibrationSession.phase == BeltCalibrationSession::Phase::Clearing) return;
      bool ended = beltCalibration.seen() &&
                   now - beltCalibration.lastSampleMs() > BeltCalibrationLimits::maximumGapMs;
      const bool timedOut = beltCalibrationSession.phase == BeltCalibrationSession::Phase::Recording &&
                           now - taskStartedMs > BeltCalibrationSessionConfig::passTimeoutMs;
      if (ended || beltCalibration.failed() || timedOut) {
        float v = 0;
        const bool accepted = beltCalibration.finish(v) && beltCalibrationSession.record(v);
        if (!accepted) ++beltCalibrationSession.rejected;
        printVelocityCalibrationReport(accepted ? "RESULT" : "REJECTED");
        char message[224];
        snprintf(message, sizeof(message),
                 "[CAL] Pass %s; accepted=%u/%u rejected=%u meanVy=%.3fmm/s SD=%.3fmm/s\n",
                 accepted ? "ACCEPTED" : "REJECTED", beltCalibrationSession.accepted,
                 BeltCalibrationSessionConfig::requiredRuns, beltCalibrationSession.rejected,
                 beltCalibrationSession.accepted ? beltCalibrationSession.mean : double(NAN),
                 beltCalibrationSession.standardDeviation());
        Serial.print(message);
        if (beltCalibrationSession.complete()) {
          velocityCalibrated = mainTracker.setVelocity(float(beltCalibrationSession.mean));
          snprintf(message, sizeof(message),
                   "[CAL] %s runs=%u rejected=%u vy=%.3fmm/s SD=%.3fmm/s minVy=%.3f maxVy=%.3fmm/s\n",
                   velocityCalibrated ? "COMPLETE" : "FAILED", beltCalibrationSession.accepted,
                   beltCalibrationSession.rejected, beltCalibrationSession.mean,
                   beltCalibrationSession.standardDeviation(), beltCalibrationSession.minimum,
                   beltCalibrationSession.maximum);
          Serial.print(message);
          Serial.println("[BELT] Speed ready in RAM. SAVE ALL in TUNING & STORAGE to persist; recalibrate if speed changes");
          finishCalibrationStream();
        } else {
          beltCalibrationSession.waitForClear();
          Serial.println("[CAL] Remove the moving tag from the ROI; wait for the next PLACE TAG prompt");
        }
      }
    }
    if (calPhase == CalPhase::Ending && takeAck(CMD_END_STREAM))
      currentState = CALIBRATION_MENU;
}

void startImaginaryPart(uint32_t now) {
  Part imaginary;
  imaginary.trackId = 1;
  imaginary.inUse = true;
  imaginary.hits = 3;
  imaginary.classId = trajectoryTestPartId;
  imaginary.boxWidth = trajectoryTestTallPart ? 20 : 40;
  imaginary.boxHeight = trajectoryTestTallPart ? 40 : 20;
  imaginary.x = trajectoryTestXmm;
  imaginary.y = TrajectoryTest::initialYmm;
  imaginary.stateMs = imaginary.observedMs = now;
  const Destination *destination =
      Classification::destinationForPart(trajectoryTestConfig, imaginary.classId);
  if (!destination) {
    trajectoryTestRun = TrajectoryTestRun::Ready;
    Serial.println("[TEST] Imaginary part ID ignored: no enabled bin mapping");
    return;
  }
  arm.start(imaginary, TrajectoryTest::velocityMmS, destination->pose,
            trajectoryTestConfig, armFeedback, now, &trajectoryTestProfiles);
  trajectoryTestRun = TrajectoryTestRun::Picking;
  char message[192];
  snprintf(message, sizeof(message),
           "[TEST] Imaginary part released: X=%.1f, Y=%.0f mm, velocity=%.0f mm/s; "
           "pickup TCP Z=%.2f mm; part ID=%u -> bin=%u\n",
           trajectoryTestXmm, TrajectoryTest::initialYmm, TrajectoryTest::velocityMmS,
           trajectoryTestConfig.pickupZ, unsigned(trajectoryTestPartId),
           unsigned(trajectoryTestBinId));
  Serial.print(message);
}

void serviceAutomaticMode(uint32_t now, uint16_t pressed) {
  // These are processed tracker snapshots, not a count of every UDP datagram.
  static uint32_t visionUpdates = 0, lastVisionLogMs = 0;
  static uint8_t visionSample = 0;
  if (currentState != lastState) {
    visionUpdates = 0;
    arm.cancel();
    mainTracker.clear();
    newLatest = false;
    streamStarted = false;
    stopRequested = exitRequested = false;
    autoLastStage = "";
    beginStreamEnd(AutoRun::SyncStop);
    lastState = currentState;
    autoTuningLaunchPending=pickupSaveRequested=false;
    if(currentState==AUTO_TUNING) {
      tuningDraft=activeTuning;pickupTuningProfile=pickupTuningRow=0;
      pickupLastX=pickupLastY=0;pickupRepeatMs=now+500;
      pickupEditorLocked=true;pickupEditorNeedsNeutral=true;
      Serial.println("[AUTO TUNING] ID IGNORANT | MODEL ID 131 | MOVE ARM");
      Serial.println("[AUTO TUNING] IDLE only: left stick=row/value; L2=WIDE/TALL; X=apply; R1=save and stop");
    }
    Serial.println("[AUTO] A: start; Y: finish assigned part/HOME and stop; "
                   "L1: finish assigned part/HOME and exit; R3: finish/HOME then tuning menu");
    Serial.println("[AUTO] L3 alone while STOPPED: reset all displayed counters");
    char message[176];
    snprintf(message, sizeof(message),
             "[AUTO] Pickup X=%ld..%ldmm; WIDE/TALL budgets=%lu/%lums; shared TCP fit/tilt, "
             "q4 tolerance=%.2fdeg; real calibrated belt velocity\n",
             PickMotion::minimumXmm, PickMotion::maximumXmm,
             (unsigned long)(activeTuning.marginMs+activeTuning.descentMs),
             (unsigned long)(activeTuning.tall.marginMs+activeTuning.tall.descentMs),
             PickMotion::wristArrivalToleranceDeg);
    Serial.print(message);
    Serial.println(!autoMovesArm() ? "[AUTO] ONLY VISION; press A to start camera tracking"
                                     : "[AUTO] MOVE ARM; press A to start");
    if (autoRespectsId()) {
      snprintf(message, sizeof(message),
               "[AUTO] ID classification waits for %u changed-position samples over >=%lums with the same ID\n",
               unsigned(CameraTracking::ClassificationConfidence::minimumSamples),
               (unsigned long)CameraTracking::ClassificationConfidence::minimumStableMs);
      Serial.print(message);
    }
    Serial.println("[BELT] Fixed AprilTag speed; AUTO cannot detect conveyor speed changes. "
                   "Operator must stop and recalibrate whenever actual speed changes.");
    snprintf(message, sizeof(message), "[AUTO] Profiles stay live until descent; camera X offsets WIDE=%+.2f TALL=%+.2fmm\n",
             mainTracker.cameraXOffsetMm(false),mainTracker.cameraXOffsetMm(true));
    Serial.print(message);
  }
  if ((pressed & BTN_R3) && !(pressed & BTN_L1)) {
    tuningMenuRequested=true; tuningReturnState=currentState; stopRequested=true;
    Serial.println("[TUNE] Requested; finish assigned part/HOME and stop stream before opening menu");
  }
  if (pressed & (BTN_Y | BTN_L1))
    stopRequested = true;
  if (pressed & BTN_L1) {
    exitRequested = true; tuningMenuRequested=pickupSaveRequested=false;
  }
  if (autoRun == AutoRun::SyncStop || autoRun == AutoRun::Ending) {
    if (takeAck(CMD_END_STREAM)) {
      streamStarted = false;
      newLatest = false;
      mainTracker.clear();
      autoRun = AutoRun::Stopped;
      char message[224];
      snprintf(message, sizeof(message),
               "[AUTO] Stopped; totals sorted=%lu missed=%lu rejected=%lu ignored=%lu "
               "outsidePickup=%lu visionUpdates=%lu\n",
               (unsigned long)sortedParts, (unsigned long)auto_parts_missed,
               (unsigned long)rejectedParts, (unsigned long)ignoredParts,
               (unsigned long)outsidePickupParts, (unsigned long)visionUpdates);
      Serial.print(message);
    }
  } else if (autoRun == AutoRun::Starting && takeAck(CMD_START_STREAM)) {
    streamStarted = true;
    newLatest = false;
    mainTracker.clear();
    visionUpdates = 0;
    visionSample = 0;
    lastVisionLogMs = now;
    Serial.println("[VISION] START_STREAM acknowledged; waiting for DATA_OBJECTS");
    if (stopRequested)
      beginStreamEnd(AutoRun::Ending);
    else if (autoMovesArm()) {
      arm.prepare(autoConfig, armFeedback, now);
      autoRun = AutoRun::Preparing;
    } else
      autoRun = AutoRun::Running;
  }
  // Reset only at a stopped cycle boundary. Do not interpret L3+R3 (or another
  // held button) as a reset, and never alter motion, tracks or stored settings.
  const bool counterResetRequested=(pressed & BTN_L3) && !((pressed | currentControlCMD.buttons) & ~BTN_L3);
  if (counterResetRequested) {
    if (autoRun == AutoRun::Stopped && !arm.busy() && !streamStarted &&
        !pendingESPmsg.pending && !pickupSaveRequested && !tuningMenuRequested && !exitRequested) {
      sortedParts = rejectedParts = ignoredParts = outsidePickupParts = 0;
      auto_parts_missed = 0;
      visionUpdates = 0;
      visionSample = 0;
      lastVisionLogMs = now;
      Serial.println("[AUTO] Counters reset; totals sorted=0 missed=0 rejected=0 ignored=0 outsidePickup=0 visionUpdates=0");
    } else
      Serial.println("[AUTO] Counter reset requires STOPPED; press Y to finish/HOME and stop, then L3");
  }
  // Real tracking continues while picking, returning HOME, or finishing Stop.
  // Its clock is never frozen: the physical conveyor keeps moving.
  if (streamStarted) {
    if (newLatest) {
      newLatest = false;
      mainTracker.updateFrame(latestReceived, totalObjects, visionTimestampMs, camera_calib);
      ++visionUpdates;
    }
    mainTracker.tick(now);
    if (!autoMovesArm() && uint32_t(now - lastVisionLogMs) >= 500) {
      lastVisionLogMs = now;
      if (!visionUpdates) {
        Serial.println("[VISION] No DATA_OBJECTS since start; check ESP [VISION ESP] status");
      } else if (uint32_t(now - visionTimestampMs) > 1000) {
        char message[128];
        snprintf(message, sizeof(message),
                 "[VISION] updates=%lu lastFrameAge=%lums; no recent DATA_OBJECTS\n",
                 (unsigned long)visionUpdates, (unsigned long)(now - visionTimestampMs));
        Serial.print(message);
      } else {
        char ids[48] = {0}, message[240];
        size_t used = 0;
        for (uint8_t i = 0; i < totalObjects; ++i)
          used += snprintf(ids + used, sizeof(ids) - used, "%s%u", i ? "," : "",
                           unsigned(latestReceived[i].ID));
        unsigned candidates = 0;
        Part candidate;
        for (int slot = 0; slot < MAX_PARTS; ++slot)
          if (mainTracker.candidate(slot, candidate, now)) ++candidates;
        if (totalObjects) {
          const uint8_t sample = visionSample++ % totalObjects;
          const objectVision &d = latestReceived[sample];
          float x, y;
          pixelTransformation(d.xCenter, d.yCenter, camera_calib, x, y);
          x += mainTracker.cameraXOffsetMm(d.height>d.width);
          snprintf(message, sizeof(message),
                   "[VISION] updates=%lu age=%lums boxes=%u IDs=[%s] sample=%u "
                   "pixel={%u,%u} world={%.2f,%.2f}mm candidates=%u\n",
                   (unsigned long)visionUpdates, (unsigned long)(now - visionTimestampMs),
                   unsigned(totalObjects), ids, unsigned(sample + 1),
                   unsigned(d.xCenter), unsigned(d.yCenter), x, y, candidates);
        } else {
          snprintf(message, sizeof(message), "[VISION] updates=%lu boxes=0 candidates=%u\n",
                   (unsigned long)visionUpdates, candidates);
        }
        Serial.print(message);
      }
    }

  } else
    newLatest = false;
  Part orientationObservation;
  if (mainTracker.observation(arm.trackId(), orientationObservation))
    arm.updateObservation(orientationObservation);
  arm.tick(armFeedback, now);
  printArmProgress(now, false);
  uint32_t id = 0;
  PickOutcome result = arm.takeOutcome(id);
  if (result == PickOutcome::Fault) {
    printArmProgress(now, true);
    latchEmergency(arm.fault());
    return;
  }
  if (result == PickOutcome::HomeReady)
    autoRun = AutoRun::Running;
  if (result == PickOutcome::Missed) {
    ++auto_parts_missed;
    mainTracker.retire(id);
    autoRun = AutoRun::Running; // Controller has already confirmed recovery HOME.
    if (arm.clawMayHoldPart()) {
      stopRequested = true;
      Serial.println("[AUTO] Missed pickup at HOME with claw CLOSED; operator intervention required. "
                     "Secure/remove any held part; A acknowledges and opens before restarting");
    } else
      Serial.println("[AUTO] Missed pickup recovered at HOME; ready for next part");
  }
  if (result == PickOutcome::Sorted) {
    ++sortedParts;
    mainTracker.retire(id);
    // Finish the whole cycle at HOME, including a recovered missed pickup.
    // No new assignment is possible until this supervised return completes.
    const AutoConfig finishing=arm.configuration();
    if (!armConfigurationReady(finishing, false)) {
      latchEmergency("Cannot return HOME: feedback, limits or configuration unavailable");
      return;
    }
    if (!arm.prepare(finishing, armFeedback, now)) {
      printArmProgress(now, true);
      latchEmergency("Cannot start AUTO return HOME");
      return;
    }
    autoRun = AutoRun::ReturningHome;
    Serial.println("[AUTO] Part sorted; returning HOME");
  }
  if (result == PickOutcome::Rejected) {
    printPlanRejection();
    ++rejectedParts;
    mainTracker.retire(id);
    const AutoConfig finishing=arm.configuration();
    if (!arm.prepare(finishing, armFeedback, now)) {
      latchEmergency("Cannot return HOME after rejected pickup");
      return;
    }
    autoRun = AutoRun::ReturningHome;
  }
  if (stopRequested && !arm.busy() &&
      (autoRun == AutoRun::Running || autoRun == AutoRun::Preparing))
    beginStreamEnd(AutoRun::Ending);
  // Consume the whole stick-click gesture, then require neutral axes, so
  // clicking L3 to reset counters cannot also adjust a tuning draft.
  const bool counterClickActive=((pressed | currentControlCMD.buttons) & BTN_L3)!=0;
  if(currentState==AUTO_TUNING && counterClickActive) {
    pickupEditorNeedsNeutral=true;pickupLastX=pickupLastY=0;
  }
  const bool editing=currentState==AUTO_TUNING && (counterClickActive || servicePickupTuning(now,pressed));
  if (autoRun == AutoRun::Stopped) {
    if(pickupSaveRequested && !arm.busy() && !pendingESPmsg.pending) {
      savePickupTuning();return;
    }
    if (tuningMenuRequested && !arm.busy()) {currentState=TUNING_MENU;return;}
    if (exitRequested) {
      currentState = currentState==AUTO_TUNING ? CALIBRATION_MENU:AUTO_MENU;
      return;
    }
    // Stop/exit take precedence over A in the same controller batch.
    if ((pressed & BTN_A) && !(pressed & (BTN_Y | BTN_L1)) && !editing) {
      if (const char *reason = autoReadinessIssue())
        printReadinessFailure("AUTO", reason, autoConfig);
      else {
        stopRequested = exitRequested = false;
        mainTracker.clear();
        newLatest = false;
        sendCriticalMessage(CMD_START_STREAM);
        autoRun = AutoRun::Starting;
        Serial.println("[VISION] START_STREAM requested; waiting for acknowledgement");
      }
    }
  }
  if (autoRun == AutoRun::Running && !stopRequested && autoMovesArm() && !arm.busy() && !editing)
    selectNextPart(now);
  printArmProgress(now, false);
  const char *stage = arm.phaseName();
  if (!autoMovesArm()) {
    switch (autoRun) {
    case AutoRun::SyncStop: stage = "VISION SYNC"; break;
    case AutoRun::Stopped: stage = "VISION STOPPED"; break;
    case AutoRun::Starting: stage = "VISION STARTING"; break;
    case AutoRun::Ending: stage = "VISION STOPPING"; break;
    default: stage = "VISION TRACKING"; break;
    }
  }
  if (strcmp(stage, autoLastStage)) {
    Serial.print("[AUTO] Stage: ");
    Serial.println(stage);
    autoLastStage = stage;
  }
}

bool sendJointCalibrationLine(const char *line) {
  const size_t n = strlen(line);
  return Serial1.write((const uint8_t *)line, n) == n;
}
void printJointCalibrationLine(const char *line) { Serial.print(line); }

void serviceJointCalibration(uint32_t now, uint16_t pressed) {
  static int8_t previousY = 0, previousX = 0;
  if (currentState != lastState) {
    arm.cancel(); mainTracker.clear(); newLatest = false;
    previousY = previousX = 0;
    jointWizard.begin(now, (now ^ (uint32_t(micros()) << 1)) | 1U);
    jointWizard.printSelection();
    Serial.println("[JOINT CAL] All reference angles are zero; use the same level placements each time");
    Serial.println("[JOINT CAL] Lead-in/sweep/stability settings: Nucleo Core/Inc/JointCalibrationConfig.h");
    lastState = currentState;
  }
  jointWizard.tick(now);
  if (jointWizard.failure) {
    latchEmergency(jointWizard.failure); return;
  }
  if (!jointWizard.active) {
    currentState = MANUAL_0; return;
  }
  // Commit is deliberately quiet on the Nucleo UART. Controller RX remains live.
  if (jointWizard.committing()) return;
  if (pressed & BTN_L1) jointWizard.cancel(now);
  else if (pressed & BTN_Y) jointWizard.stop(now);
  else if (pressed & BTN_X) jointWizard.accept(now);
  else if (pressed & BTN_R1) jointWizard.save(now);
  else if (pressed & BTN_A) jointWizard.start(now);
  else {
    int8_t y = currentControlCMD.jsLy, x = currentControlCMD.jsLx;
    if (!jointWizard.pending) {
      if (y && y != previousY)
        jointWizard.select(unsigned(constrain(int(jointWizard.axis) + y, 0, 2)), jointWizard.direction, now);
      else if (x && x != previousX) jointWizard.select(jointWizard.axis, x > 0 ? 1 : -1, now);
    }
    previousY = y; previousX = x;
  }
  if (pcInputExists && !jointWizard.pending) {
    if (!strcmp(pcInputBuffer, "JC:EXPORT")) jointWizard.command("EXPORT", nullptr, now);
    else if (!strcmp(pcInputBuffer, "JC:RESTORE")) jointWizard.command("RESTORE", nullptr, now);
    else if (!strncmp(pcInputBuffer, "JC:IMPORT,", 10)) jointWizard.command("IMPORT", pcInputBuffer + 9, now);
  }
}

void serviceHomographyCalibration(uint32_t now,uint16_t pressed) {
    if (currentState != lastState) {
      Serial.println("[CAL] Homography: leave only the four stationary corner tags 0..3; remove moving tags");
      cameraCancelRequested=false;
      homographyDone = false;
      camera_calib.valid = false;
      mainTracker.clear();
      newLatest = false;
      sendCriticalMessage(CMD_START_STREAM);
      calPhase = CalPhase::Starting;
      lastState = currentState;
    }
    if (calPhase == CalPhase::Starting && takeAck(CMD_START_STREAM)) {
      calPhase = CalPhase::Collecting;
      taskStartedMs = now;
      newLatest = false;
    }
    if ((pressed & BTN_L1) && calPhase==CalPhase::Starting) cameraCancelRequested=true;
    if ((pressed & BTN_L1) && calPhase==CalPhase::Collecting) cameraCancelRequested=true;
    if (cameraCancelRequested && calPhase==CalPhase::Collecting) finishCalibrationStream();
    if (calPhase == CalPhase::Collecting) {
      if (newLatest) {
        newLatest = false;
        float u[4]{}, v[4]{};
        uint8_t seen[4]{};
        for (int i = 0; i < totalObjects; ++i)
          if (latestReceived[i].ID < 4) {
            int j = latestReceived[i].ID;
            ++seen[j];
            u[j] = latestReceived[i].xCenter;
            v[j] = latestReceived[i].yCenter;
          }
        bool complete = true;
        for (uint8_t n : seen)
          if (n != 1)
            complete = false;
        if (complete) {
          calibH c = AutoHomography(u, v);
          if (c.valid) {
            camera_calib = c;
            homographyDone = true;
            finishCalibrationStream();
          }
        }
      }
      if (calPhase == CalPhase::Collecting && now - taskStartedMs > 12000) {
        Serial.println("[CAL] Four valid, unique tags were not obtained");
        finishCalibrationStream();
      }
    }
    if (calPhase == CalPhase::Ending && takeAck(CMD_END_STREAM)) {
      Serial.println(homographyDone ? "[CAL] Homography ready in RAM; existing belt speed retained. SAVE ALL in TUNING & STORAGE to persist"
                                    : "[CAL] Homography failed");
      currentState = CALIBRATION_MENU;
    }

}

SavedRobotSettings snapshotRuntimeSettings() {
  SavedRobotSettings s; s.tuning=activeTuning;
  s.homography=camera_calib; s.homography.valid=homographyDone && camera_calib.valid;
  s.velocityValid=velocityCalibrated && mainTracker.calibrated(); s.velocity=mainTracker.velocity();
  return s;
}
void installRuntimeSettings(const SavedRobotSettings &s) {
  activeTuning=tuningDraft=s.tuning; activeTuning.apply(autoConfig);
  mainTracker.setCameraXOffsetsMm(activeTuning.partXOffsetMm,activeTuning.tall.partXOffsetMm);
  camera_calib=s.homography; homographyDone=camera_calib.valid;
  velocityCalibrated=mainTracker.setVelocity(s.velocityValid ? s.velocity:0);
  newLatest=false;
}
void printRuntimeStorageStatus() {
  char b[256];
  snprintf(b,sizeof(b),"[STORE] available=%u generation=%lu homography=%u velocity=%u vy=%.3fmm/s status=%s\n",
      settingsStore.available,(unsigned long)settingsStore.generation,homographyDone,velocityCalibrated,
      mainTracker.velocity(),settingsStore.error ? settingsStore.error:"OK");
  Serial.print(b);
  Serial.println("[CAL] Operator responsibility: recalibrate homography if camera moves; belt velocity if actual speed changes (including after power-on). AUTO cannot verify either");
}
void loadRuntimeSettings() {
  SavedRobotSettings saved;
  if(settingsStore.load(saved)) {
    installRuntimeSettings(saved);
    Serial.println("[STORE] Tuning + valid camera/belt calibrations restored from QSPI Flash");
  } else Serial.println("[STORE] No usable record loaded; current RAM/default settings retained");
  printRuntimeStorageStatus();
}
void printTuningRow() {
  char b[256];
  if(tuningRow>=TuningMenu::Apply)
    snprintf(b,sizeof(b),"[TUNE] %u/%u %s -- press X\n",tuningRow+1,
             unsigned(TuningMenu::Count),TuningMenu::name(tuningRow));
  else
    snprintf(b,sizeof(b),"[TUNE] %u/%u %s: draft=%.3f active=%.3f\n",tuningRow+1,
      unsigned(TuningMenu::Count),TuningMenu::name(tuningRow),TuningMenu::value(tuningDraft,tuningRow),
      TuningMenu::value(activeTuning,tuningRow));
  Serial.print(b);
}
bool applyTuningDraft() {
  if(const char *issue=tuningDraft.issue()) {Serial.print("[TUNE] Rejected: ");Serial.println(issue);return false;}
  if(arm.busy()) {Serial.println("[TUNE] Wait until the assigned cycle finishes");return false;}
  activeTuning=tuningDraft; activeTuning.apply(autoConfig);
  const bool offsetChanged=mainTracker.cameraXOffsetMm(false)!=activeTuning.partXOffsetMm ||
      mainTracker.cameraXOffsetMm(true)!=activeTuning.tall.partXOffsetMm;
  mainTracker.setCameraXOffsetsMm(activeTuning.partXOffsetMm,activeTuning.tall.partXOffsetMm);
  if(offsetChanged) {
    newLatest=false;
    Serial.println("[TUNE] Camera X offset changed; old tracks cleared, homography and belt speed retained");
  }
  Serial.println("[TUNE] Applied to RAM for subsequent cycles; SAVE ALL to survive reboot");
  return true;
}
void printPickupTuningRow() {
  char b[256];
  const unsigned rows=PickupTuningMenu::count(pickupTuningProfile);
  if(pickupTuningRow>=rows)pickupTuningRow=rows-1;
  snprintf(b,sizeof(b),"[PICK TUNE] %s %u/%u %s: draft=%.3f active=%.3f\n",
    pickupTuningProfile ? "TALL height>width / yaw90":"WIDE height<=width / yaw0",
    pickupTuningRow+1,rows,PickupTuningMenu::name(pickupTuningRow),
    PickupTuningMenu::value(tuningDraft.profile(pickupTuningProfile),pickupTuningRow),
    PickupTuningMenu::value(activeTuning.profile(pickupTuningProfile),pickupTuningRow));
  Serial.print(b);
  if(pickupTuningRow==PickupTuningMenu::Id1ExtraX)
    Serial.println("[PICK TUNE] Additive correction for stable ID 1 + TALL in ID classification AUTO only; positive moves toward larger X. ID IGNORANT and synthetic tests do not apply it");
}
bool servicePickupTuning(uint32_t now,uint16_t pressed) {
  const bool editable=!arm.busy() && !pendingESPmsg.pending &&
      (autoRun==AutoRun::Stopped || autoRun==AutoRun::Running) && !pickupSaveRequested;
  if(!editable) {
    if(!pickupEditorLocked)Serial.println("[PICK TUNE] Locked until the assigned cycle finishes at HOME/IDLE");
    pickupEditorLocked=true;pickupEditorNeedsNeutral=true;
    pickupLastX=pickupLastY=0;pickupRepeatMs=now+500;
    return false;
  }
  if(pickupEditorLocked) {
    pickupEditorLocked=false;
    Serial.println("[PICK TUNE] IDLE: edits available; 10ms / 0.5mm steps. X applies both profiles; R1 saves all and stops");
    printPickupTuningRow();
  }
  if(!udpControllerStable)return true;
  const int8_t y=currentControlCMD.jsLy,x=currentControlCMD.jsLx;
  if(pickupEditorNeedsNeutral) {
    if(x || y)return true;
    pickupEditorNeedsNeutral=false;
  }
  if(pressed & (BTN_Y | BTN_L1 | BTN_R3))return true;
  bool editing=x || y || (pressed & (BTN_L2 | BTN_X | BTN_R1));
  if(pressed & BTN_L2) {pickupTuningProfile^=1;printPickupTuningRow();}
  const bool repeat=(y || x) && int32_t(now-pickupRepeatMs)>=0;
  if(y && (y!=pickupLastY || repeat)) {
    int row=int(pickupTuningRow)+y;
    const unsigned rows=PickupTuningMenu::count(pickupTuningProfile);
    pickupTuningRow=row<0 ? 0:unsigned(row)>=rows ? rows-1:unsigned(row);
    printPickupTuningRow();pickupRepeatMs=now+(y==pickupLastY ? 200:500);
  } else if(x && (x!=pickupLastX || repeat)) {
    PickupTuningMenu::adjust(tuningDraft.profile(pickupTuningProfile),pickupTuningRow,x);
    printPickupTuningRow();pickupRepeatMs=now+(x==pickupLastX ? 200:500);
  }
  pickupLastX=x;pickupLastY=y;
  if(pressed & BTN_X) {applyTuningDraft();printPickupTuningRow();}
  if(pressed & BTN_R1) {
    if(const char *issue=tuningDraft.issue()) {Serial.print("[PICK TUNE] Cannot save: ");Serial.println(issue);}
    else {
      pickupSaveRequested=true;stopRequested=true;
      Serial.println("[STORE] Save requested at IDLE; shutting down camera stream first. A will restart afterward");
    }
  }
  return editing;
}
void savePickupTuning() {
  pickupSaveRequested=false;
  if(arm.busy() || autoRun!=AutoRun::Stopped || streamStarted || pendingESPmsg.pending)return;
  if(!applyTuningDraft())return;
  Serial1.print("\nS:MODES\n");
  if(!settingsStore.available) {SavedRobotSettings previous;settingsStore.load(previous);}
  if(settingsStore.save(snapshotRuntimeSettings()))Serial.println("[STORE] Both pickup profiles + calibrations verified in Flash; stopped, A restarts");
  else {Serial.print("[STORE] Save failed; RAM remains active: ");Serial.println(settingsStore.error ? settingsStore.error:"Unavailable");}
  printRuntimeStorageStatus();
}
void serviceTuningMenu(uint32_t now,uint16_t pressed) {
  if(currentState!=lastState) {
    if(arm.busy()) {latchEmergency("Tuning menu entered during motion");return;}
    tuningMenuRequested=false; tuningDraft=activeTuning;
    tuningRow=0; tuningLastX=tuningLastY=0; tuningRepeatMs=now+500;
    Serial1.print("\nS:MODES\n"); // No arm motion/automatic restart while editing or writing Flash.
    lastState=currentState;
    Serial.println("[TUNE] Left stick UP/DOWN: row; LEFT/RIGHT: value. Timing step=10ms; position step=0.5mm");
    Serial.println("[TUNE] Pickup settings moved to CALIBRATION MENU -> AUTO PICKUP TUNING (separate WIDE/TALL profiles)");
    Serial.println("[TUNE] X: selected action; A: apply draft; L1: back/discard draft. Slope and gripper angles are fixed");
    Serial.println("[TUNE] SAVE ALL stores applied tuning + currently valid camera/belt calibrations. New calibrations need explicit SAVE ALL");
    printRuntimeStorageStatus(); printTuningRow();
  }
  if(!udpControllerStable) {tuningLastX=tuningLastY=0;tuningRepeatMs=now+500;return;}
  if(pressed & BTN_L1) {
    tuningDraft=activeTuning; currentState=tuningReturnState;
    Serial.println("[TUNE] Back; applied RAM settings retained. Press A in AUTO/TEST to start again");return;
  }
  const int8_t y=currentControlCMD.jsLy,x=currentControlCMD.jsLx;
  bool repeat=(y || x) && int32_t(now-tuningRepeatMs)>=0;
  if(y && (y!=tuningLastY || repeat)) {
    int next=int(tuningRow)+y; tuningRow=next<0 ? 0:next>=TuningMenu::Count ? TuningMenu::Count-1:next;
    printTuningRow(); tuningRepeatMs=now+(y==tuningLastY ? 200:500);
  } else if(x && (x!=tuningLastX || repeat) && tuningRow<TuningMenu::Apply) {
    TuningMenu::adjust(tuningDraft,tuningRow,x);printTuningRow();
    tuningRepeatMs=now+(x==tuningLastX ? 200:500);
  }
  tuningLastX=x;tuningLastY=y;
  if(pressed & BTN_A) {applyTuningDraft();return;}
  if(!(pressed & BTN_X))return;
  if(tuningRow==TuningMenu::Apply)applyTuningDraft();
  else if(tuningRow==TuningMenu::Defaults) {tuningDraft.resetGeneralDefaults();Serial.println("[TUNE] Shared/test build defaults in draft only; pickup profiles and calibrations retained");}
  else if(tuningRow==TuningMenu::Reload) {loadRuntimeSettings();tuningDraft=activeTuning;}
  else if(tuningRow==TuningMenu::Save) {
    if(arm.busy() || pendingESPmsg.pending) {Serial.println("[STORE] Wait for motion/camera exchange to finish");return;}
    if(!applyTuningDraft())return;
    if(!settingsStore.available) {SavedRobotSettings previous;settingsStore.load(previous);}
    Serial.println("[STORE] Saving at rest; no movement can start until the operation completes");
    if(settingsStore.save(snapshotRuntimeSettings())) Serial.println("[STORE] SAVE ALL verified; settings and calibrations survive reboot");
    else {Serial.print("[STORE] Save failed; RAM remains active: ");Serial.println(settingsStore.error ? settingsStore.error:"Unavailable");}
    printRuntimeStorageStatus();
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(115200);
  jointWizard.bind(sendJointCalibrationLine, printJointCalibrationLine);
  jointMotionTest.bind(sendJointCalibrationLine, printJointCalibrationLine);

  params = robot_get_dh_params();
  Classification::configure(autoConfig, params);
  activeTuning=RuntimeTuning::defaults();
  activeTuning.apply(autoConfig);
  loadRuntimeSettings();
  arm.bind(params, {sendAutoJoints, sendAutoGripper, sendAutoHome});

  Serial.setTimeout(10);

  WiFi.config(IPAddress(192, 168, 4, 1));

  Serial.println("Creando Hotspot...");
  auto status = WiFi.beginAP(ssid, password);
  if (status != WL_AP_LISTENING) {
    Serial.println("Fallo al crear el Access Point.");
    while (true)
      ;
  }
  Serial.println("Red creada. IP: " + WiFi.localIP().toString());
  udpController.begin(udpPort_Controller);
  cameraUDP.begin(cameraUDPport);
}

// ===================================
// MAIN LOOP
// ===================================
void loop() {
  // String pcInput = "";
  pcInputBuffer[0] = '\0';
  pcInputExists = false;
  // put your main code here, to run repeatedly:
  controllerPressedEdges = 0;
  controllerEmergencyReceived = controllerTimedOutThisLoop = false;
  serviceControllerPackets();
  // Apply the same motion-mode policy before other loop work. In states
  // requiring the controller, later recovery cannot clear this stop request.
  serviceImmediateControllerFaults();

  static char pcLine[MAX_PC_BUFFER];
  static unsigned pcUsed=0; static bool pcDiscard=false;
  for(unsigned budget=0;budget<64 && Serial.available();++budget) {
    int c=Serial.read(); if(c<0) break;
    if(c=='\r') continue;
    if(c=='\n') {
      if(!pcDiscard && pcUsed) {pcLine[pcUsed]='\0';memcpy(pcInputBuffer,pcLine,pcUsed+1);pcInputExists=true;}
      pcUsed=0;pcDiscard=false;break;
    }
    if(!pcDiscard) {
      if(pcUsed>=MAX_PC_BUFFER-1) {pcDiscard=true;pcUsed=0;}
      else pcLine[pcUsed++]=char(c);
    }
  }

  readNucleoSerial();

  uint32_t visionWorkStart = millis();
  managePingsVision();
  manageVisionPort();
  uint32_t visionWorkMs = uint32_t(millis() - visionWorkStart);
  if (visionWorkMs > controllerLink.maxVisionWorkMs)
    controllerLink.maxVisionWorkMs = visionWorkMs;

  // Re-service controller RX after serial/network work, before state decisions.
  serviceControllerPackets();
  if (newControlCMD) {
    newControlCMD = false;
    currentControlCMD.buttons = currentControlCommand.buttons;
    currentControlCMD.jsLx = pocketSign(currentControlCommand.lx);
    currentControlCMD.jsRx = pocketSign(currentControlCommand.rx);
    currentControlCMD.jsLy = pocketSign(currentControlCommand.ly);
    currentControlCMD.jsRy = pocketSign(currentControlCommand.ry);
  }
  PE_buttons = controllerPressedEdges | (currentControlCMD.buttons & ~lastControlCMD.buttons);
  if (currentState != lastState)
    PE_buttons = 0;
  printPressedButtons(PE_buttons);
  lastControlCMD = currentControlCMD;

  serviceGlobalFaults();
  // Any required stop command is sent before diagnostic printing. Preserve the
  // detection-time age/sequence even if the second poll saw link recovery.
  if (controllerTimedOutThisLoop)
    activate_UDP_Failsafe();
  // Log once after supervision, rather than printing while draining packets.
  reportControllerRecovery();

  switch (currentState) {
  case TUNING_MENU: { serviceTuningMenu(millis(),PE_buttons); break; }
  case JOINT_MOTION_TEST: {
    if (currentState != lastState) {
      AutoConfig motionTestConfig=autoConfig;activeTuning.apply(motionTestConfig,1);
      jointMotionTest.enter(arm,params,motionTestConfig);
      lastState=currentState;
    }
    unsigned buttons=0;
    if (PE_buttons&BTN_A) buttons|=JointMotionTest::Start;
    if (PE_buttons&BTN_Y) buttons|=JointMotionTest::Pause;
    if (PE_buttons&BTN_L1) buttons|=JointMotionTest::Cancel;
    uint32_t now=millis();
    if (!limits_received && uint32_t(now-last_limit_req_time)>1000) {
      Serial1.print("S:REQ_LIMITS\n"); last_limit_req_time=now;
    }
    if ((buttons&JointMotionTest::Start) &&
        (jointMotionTest.state()==JointMotionTest::State::Ready ||
         jointMotionTest.state()==JointMotionTest::State::Finished)) {
      if (const char *reason=armConfigurationIssue(jointMotionTest.configuration(),true)) {
        printReadinessFailure("JOINT TEST",reason,jointMotionTest.configuration());
        buttons&=~JointMotionTest::Start;
      }
    }
    jointMotionTest.tick(armFeedback,millis(),buttons,(uint32_t(micros())^millis())|1U);
    if (jointMotionTest.fault()) latchEmergency(jointMotionTest.fault());
    else if (jointMotionTest.exitRequested()) currentState=CALIBRATION_MENU;
    break;
  }
  case JOINT_CALIBRATION: {
    serviceJointCalibration(millis(), PE_buttons);
    break;
  }
  // ===================================
  // WAITING_CONNECTION STATE DEF
  // ===================================
  case WAITING_CONNECTION: {
    // State initialization code:
    if (currentState != lastState) {
      Serial.println("[SYSTEM] State changed to: WAITING CONNECTION");
      // PE_buttons = 0;
      lastState = currentState;
    }
    // State running code:
    if (pcInputExists) {
      Serial.println("[SYSTEM] Ignoring commands in this state");
    }

    if (!limits_received && (millis() - last_limit_req_time > 1000)) {
      Serial1.print("S:REQ_LIMITS\n");
      last_limit_req_time = millis();
    }

    // State exit code:
    if (udpControllerStable && esp32IP_known) {
      if (entered_ES) {
        currentState = EMERGENCY_STOP;
      } else {
        currentState = MODE_SELECTOR;
      }
    }
    break;
  }
  // ===================================
  // MODE_SELECTOR STATE DEF
  // ===================================
  case MODE_SELECTOR: {
    // State initialization code:
    if (currentState != lastState) {
      Serial.println("[SYSTEM] State changed to: MODE SELECTOR");
      // Leading delimiter lets a restarted Nucleo receiver discard a damaged
      // command suffix without losing this explicit reset/state command.
      Serial1.print("\nS:MODES\n");
      Serial1.print("J:0,1,STATUS\n");
      Serial.println("  L1+R1 = MANUAL  |  L2+R2 = AUTO  |  L3+R3 = CALIBRATION MENU");
      lastState = currentState;
    }
    // State running code:
    if (pcInputExists) {
      Serial.println("[SYSTEM] Ignoring commands in this state");
    }

    if (!limits_received && (millis() - last_limit_req_time > 1000)) {
      Serial1.print("S:REQ_LIMITS\n");
      last_limit_req_time = millis();
    }

    // Wait for fresh controller traffic before accepting a menu selection.
    if (!udpControllerStable) break;
    // State exit code:
    if (currentControlCMD.buttons & BTN_B) {
      entered_ES = true;
      currentState = EMERGENCY_STOP;
    } else if ((PE_buttons & BTN_L1) && (PE_buttons & BTN_R1)) {
      if (limits_received)
        currentState = MANUAL_0;
      else
        Serial.println("[WARN] No limits in workspace, check Nucleo");
    } else if ((PE_buttons & BTN_L2) && (PE_buttons & BTN_R2)) {
      if (!limits_received) {
        Serial.println("[WARN] No limits in workspace, check Nucleo");
      } else if (cameraOffline) {
        Serial.println("[WARN] Camera Offline");
      } else if (!homographyDone) {
        Serial.println("[WARN] No homography done");
      } else if (!velocityCalibrated) {
        Serial.println("[WARN] No belt velocity calibrated");
      } else {
        currentState = AUTO_MENU;
      }
    } else if ((PE_buttons & BTN_L3) && (PE_buttons & BTN_R3)) {
      currentState = CALIBRATION_MENU;
    }
    break;
  }
  // ===================================
  // EMERGENCY_STOP STATE DEF
  // ===================================
  case EMERGENCY_STOP: {
    if (currentState != lastState) {
      entered_ES = true;
      leave_ES = false;
      holdingResetCombo = false;
      failed_ik = false;
      arm.cancel();
      mainTracker.clear();
      pendingESPmsg.pending = false;
      messageAck = false;
      Serial1.print("S:ESTOP\n");
      lastState = currentState;
      Serial.println("[SYSTEM] Emergency latched; reset requires a 3-second hold");
    }
    bool reset = ((currentControlCMD.buttons & BTN_L1) && (currentControlCMD.buttons & BTN_R2)) ||
                 ((currentControlCMD.buttons & BTN_R1) && (currentControlCMD.buttons & BTN_L2));
    reset = reset && udpControllerStable && !controllerEmergencyReceived &&
            !controllerTimedOutThisLoop && !(currentControlCMD.buttons & BTN_B);
    if (!reset)
      holdingResetCombo = false;
    else if (!holdingResetCombo) {
      holdingResetCombo = true;
      resetComboStartTime = millis();
    } else if (millis() - resetComboStartTime >= 3000) {
      entered_ES = false;
      communicationError = false;
      packetMissed = false;
      holdingResetCombo = false;
      streamStarted = false;
      newLatest = false;
      currentState = MODE_SELECTOR;
    }
    break;
  }
  // ===================================
  // MANUAL_0 STATE DEF
  // ===================================
  case MANUAL_0: {
    // State initialization code:
    if (currentState != lastState) {
      Serial.println("[SYSTEM] State changed to: MANUAL 0");
      Serial.println("[JOINT CAL] Hold L3+R3 for 1 second with neutral controls to enter calibration");
      jointCalibrationEntryHeld = false;
      Serial1.print("S:MANUAL\n");
      Serial1.print("V:500\n");
      global_velocity = 500;
      Serial1.print("T:0\n");
      Serial1.print("M:0,0,0,0,0,0\n");
      // PE_buttons = 0;
      auto btns = currentControlCMD.buttons;
      int8_t current_move[7] = {currentControlCMD.jsLx,
                                currentControlCMD.jsRx,
                                currentControlCMD.jsLy,
                                currentControlCMD.jsRy,
                                (int8_t)(!!(btns & BTN_L2) - !!(btns & BTN_R2)),
                                (int8_t)(!!(btns & BTN_A) - !!(btns & BTN_Y)),
                                0};
      memcpy(last_move, current_move, sizeof(last_move));
      lastState = currentState;
    }
    // Calibration entry consumes no normal movement command.
    {
      const uint16_t chord = BTN_L3 | BTN_R3;
      bool neutral = !currentControlCMD.jsLx && !currentControlCMD.jsLy &&
          !currentControlCMD.jsRx && !currentControlCMD.jsRy &&
          !(currentControlCMD.buttons & ~(chord));
      if ((currentControlCMD.buttons & chord) == chord && neutral) {
        if (!jointCalibrationEntryHeld) { jointCalibrationEntryHeld = true; jointCalibrationEntryMs = millis(); }
        if (uint32_t(millis() - jointCalibrationEntryMs) >= 1000) {
          Serial1.print("M:0,0,0,0,0,0\n"); currentState = JOINT_CALIBRATION; break;
        }
      } else jointCalibrationEntryHeld = false;
    }
    // State running code:
    // Serial commands parsing
    if (pcInputExists) {
      if (!strcmp(pcInputBuffer, "JC:EXPORT")) {
        Serial1.print("J:0,1,EXPORT\n");
      } else if (strncmp(pcInputBuffer, "P:", 2) == 0) {
        float q1, q2, q3, q4, q5;
        int parsed = sscanf(pcInputBuffer, "P:%f,%f,%f,%f,%f", &q1, &q2, &q3, &q4, &q5);
        if (parsed == 5) {
          Serial1.println(pcInputBuffer);
          Serial.print("[CMD] Command sent to Nucleo: ");
          Serial.println(pcInputBuffer);
        } else {
          Serial.println("[WARN] Incorrect format, expected P:roll,pitch,j3,j4,j5");
        }
      } else {
        Serial.println("[WARN] Command not recognized or ignored in this state");
      }
    }

    // Nucleo commands parsing
    printRawAngles();

    // Controller movement
    auto btns = currentControlCMD.buttons;

    int8_t current_move[7] = {currentControlCMD.jsLx,
                              currentControlCMD.jsRx,
                              currentControlCMD.jsLy,
                              currentControlCMD.jsRy,
                              (int8_t)(!!(btns & BTN_L2) - !!(btns & BTN_R2)),
                              (int8_t)(!!(btns & BTN_A) - !!(btns & BTN_Y)),
                              0};

    if (memcmp(current_move, last_move, sizeof(current_move)) != 0) {
      char cmdBuf[24];
      char *ptr = cmdBuf;

      *ptr++ = 'M';
      *ptr++ = ':';
      for (int i = 0; i < 6; ++i) {
        int8_t val = current_move[i];
        if (val < 0) {
          *ptr++ = '-';
          *ptr++ = '1';
        } else {
          *ptr++ = (char)('0' + val);
        }
        if (i < 5)
          *ptr++ = ',';
      }
      *ptr++ = '\n';
      Serial1.write((const uint8_t *)cmdBuf, ptr - cmdBuf);

      memcpy(last_move, current_move, sizeof(last_move));
    }

    // Speed control
    int8_t dir = (!!(PE_buttons & BTN_R1) - !!(PE_buttons & BTN_L1));

    if (dir != 0) {
      // lastSpeedChangeTime = millis();
      int16_t new_vel = (int16_t)global_velocity + (dir * 50);

      if (new_vel < 100)
        new_vel = 100;
      else if (new_vel > 900)
        new_vel = 900;

      if ((uint16_t)new_vel != global_velocity) {
        global_velocity = (uint16_t)new_vel;
        Serial1.print("V:");
        Serial1.println(global_velocity);
      }
    }

    // State exit code:
    if (!udpControllerStable) {
      currentState = EMERGENCY_STOP;
    } else if (currentControlCMD.buttons & BTN_B) {
      entered_ES = true;
      currentState = EMERGENCY_STOP;
    } else if (PE_buttons & BTN_X) {
      currentState = MANUAL_1;
    } else if (((currentControlCMD.buttons & BTN_L1) && (currentControlCMD.buttons & BTN_R2)) ||
               ((currentControlCMD.buttons & BTN_L2) && (currentControlCMD.buttons & BTN_R1))) {
      currentState = MODE_SELECTOR;
    }
    break;
  }
  // ===================================
  // MANUAL_1 STATE DEF
  // ===================================
  case MANUAL_1: {
    // State initialization code:
    if (currentState != lastState) {
      Serial1.print("S:MANUAL\n");
      Serial1.print("V:500\n");
      global_velocity = 500;
      Serial1.print("T:1\n");
      Serial1.print("P:0,0,0,0,0\n");
      Serial1.print("G:0\n");
      gripper_target = 0.0f;
      robot_fkine(HOME, params, ik_target);
      memcpy(manual_q_seed, HOME, sizeof(manual_q_seed));
      failed_ik = false;
      new_ik_target = true;
      target_step_lin = 2.5f;
      target_step_ang = 1.0f;
      // PE_buttons = 0;
      auto btns = currentControlCMD.buttons;
      int8_t current_move[7] = {(int8_t)-currentControlCMD.jsLy,
                                (int8_t)-currentControlCMD.jsLx,
                                (int8_t)(!!(btns & BTN_R3) - !!(btns & BTN_L3)),
                                (int8_t)(!!(btns & BTN_L2) - !!(btns & BTN_R2)),
                                (int8_t)-currentControlCMD.jsRy,
                                currentControlCMD.jsRx,
                                (int8_t)(!!(btns & BTN_Y) - !!(btns & BTN_A))};
      memcpy(last_move, current_move, sizeof(last_move));
      Serial.println("[SYSTEM] State changed to: MANUAL 1");
      lastState = currentState;
    }
    // State running code:

    // Nucleo commands parsing
    printRealIkTargets();

    // Controller movement
    auto btns = currentControlCMD.buttons;

    // int8_t current_move[6] = {
    //   currentControlCMD.jsLx,
    //   currentControlCMD.jsRx,
    //   currentControlCMD.jsLy,
    //   currentControlCMD.jsRy,
    //   (int8_t)(!!(btns & BTN_L2) - !!(btns & BTN_R2)),
    //   (int8_t)(!!(btns & BTN_A) - !!(btns & BTN_Y))
    // };

    int8_t current_move[7] = {(int8_t)-currentControlCMD.jsLy,
                              (int8_t)-currentControlCMD.jsLx,
                              (int8_t)(!!(btns & BTN_R3) - !!(btns & BTN_L3)),
                              (int8_t)(!!(btns & BTN_L2) - !!(btns & BTN_R2)),
                              (int8_t)-currentControlCMD.jsRy,
                              currentControlCMD.jsRx,
                              (int8_t)(!!(btns & BTN_Y) - !!(btns & BTN_A))};

    bool anyArmMotion = false;
    for (int i = 0; i < 6; i++) {
      if (current_move[i] != 0) {
        anyArmMotion = true;
        break;
      }
    }
    bool anyGripperMotion = (current_move[6] != 0);

    if ((memcmp(current_move, last_move, sizeof(current_move)) != 0)) {
      memcpy(last_move, current_move, sizeof(last_move));
    }

    if ((millis() - lastStepTime) > TARGET_STEP_DELAY) {
      lastStepTime = millis();
      if (anyArmMotion) {
        if (!feedbackFresh() || !limits_received) {
          latchEmergency("MANUAL_1 feedback or joint limits unavailable");
          break;
        }
        float increment[6], requested[6], seed[5];
        memcpy(seed, manual_q_seed, sizeof(seed));
        for (int i = 0; i < 6; i++) {
          int8_t val = current_move[i];
          increment[i] = val * (i < 3 ? target_step_lin : target_step_ang);
          requested[i] = ik_target[i] + increment[i];
        }
        IkReport report;
        failed_ik = !robot_ikine_increment(manual_q_seed, ik_target, increment, max_iterations,
                                           tolerance_mm, tolerance_rad, params, report);

        if (!failed_ik) {
          new_ik_target = true;
          char pBuf[128];
          snprintf(pBuf, sizeof(pBuf), "P:%.2f,%.2f,%.2f,%.2f,%.2f\n", manual_q_seed[1],
                   manual_q_seed[0], manual_q_seed[2], manual_q_seed[3], manual_q_seed[4]);
          Serial1.print(pBuf);
        }
        if ((failed_ik || report.fullRotationErrorRad > tolerance_rad) &&
            uint32_t(millis() - lastManualIkMessageMs) >= 500) {
          lastManualIkMessageMs = millis();
          Serial.println(failed_ik ? "[IK MANUAL] Rejected increment; previous target retained"
                                   : "[IK MANUAL] Unattainable orientation component relaxed");
          printIkDiagnostic("MANUAL", requested, seed, report);
        }
      }
      if (anyGripperMotion) {
        float temp_gripper = gripper_target + current_move[6] * 1;
        if (temp_gripper < LIMIT_GRIPPER_MIN)
          temp_gripper = LIMIT_GRIPPER_MIN;
        else if (temp_gripper > LIMIT_GRIPPER_MAX)
          temp_gripper = LIMIT_GRIPPER_MAX;

        char gBuf[32];
        if (temp_gripper != gripper_target) {
          gripper_target = temp_gripper;
          snprintf(gBuf, sizeof(gBuf), "G:%.2f\n", gripper_target);
          Serial1.print(gBuf);
        }
      }
    }

    // Speed control
    int8_t dir = (!!(PE_buttons & BTN_R1) - !!(PE_buttons & BTN_L1));

    if (dir != 0) {
      // lastSpeedChangeTime = millis();
      float new_lin = (float)target_step_lin + (dir * 0.5);
      float new_ang = (float)target_step_ang + (dir * 0.2);

      if (new_lin < 2.5)
        new_lin = 2.5;
      else if (new_lin > 7.5)
        new_lin = 7.5;

      if (new_ang < 1)
        new_ang = 1;
      else if (new_ang > 3)
        new_ang = 3;

      target_step_lin = new_lin;
      target_step_ang = new_ang;
    }

    // State exit code:
    if (!udpControllerStable) {
      currentState = EMERGENCY_STOP;
    } else if (currentControlCMD.buttons & BTN_B) {
      entered_ES = true;
      currentState = EMERGENCY_STOP;
    } else if (PE_buttons & BTN_X) {
      currentState = MANUAL_0;
    } else if (((currentControlCMD.buttons & BTN_L1) && (currentControlCMD.buttons & BTN_R2)) ||
               ((currentControlCMD.buttons & BTN_L2) && (currentControlCMD.buttons & BTN_R1))) {
      currentState = MODE_SELECTOR;
    }
    break;
  }
  // ===================================
  // AUTO_MENU STATE DEF
  // ===================================
  case AUTO_MENU: {
    if (currentState != lastState) {
      arm.cancel();
      mainTracker.clear();
      requestedCustomAlgo = requestedTagsAlgo = requestedAlgoVelocity = false;
      int8_t m[7] = {(int8_t)-currentControlCMD.jsLy,
                     (int8_t)-currentControlCMD.jsLx,
                     (int8_t)-currentControlCMD.jsRy,
                     0,
                     0,
                     0,
                     0};
      memcpy(last_move, m, sizeof(m));
      printAutoMenu();
      lastState = currentState;
      Serial.println("[MENU] X: select; configure class destinations before ID sorting");
    }
    if (!udpControllerStable) break;
    int8_t m[7] = {(int8_t)-currentControlCMD.jsLy,
                   (int8_t)-currentControlCMD.jsLx,
                   (int8_t)-currentControlCMD.jsRy,
                   0,
                   0,
                   0,
                   0};
    if (memcmp(m, last_move, sizeof(m))) {
      menu_option0 = uint8_t(constrain(int(menu_option0) + m[0], 0, 1));
      menu_option1 = uint8_t(constrain(int(menu_option1) + m[1], 0, 1));
      menu_option2 = uint8_t(constrain(int(menu_option2) + m[2], 0, 1));
      memcpy(last_move, m, sizeof(m));
      printAutoMenu();
    }
    if (PE_buttons & BTN_X) {
      requestedCustomAlgo = true;
      currentState = WAITING_ALGO_SWITCH;
    } else if (((currentControlCMD.buttons & BTN_L1) && (currentControlCMD.buttons & BTN_R2)) ||
               ((currentControlCMD.buttons & BTN_L2) && (currentControlCMD.buttons & BTN_R1)))
      currentState = MODE_SELECTOR;
    break;
  }
  // ===================================
  // AUTO_MODE STATE DEF
  // ===================================
  case AUTO_MODE:
  case AUTO_TUNING: {
    serviceAutomaticMode(millis(), PE_buttons);
    break;
  }
  // ===================================
  // CALIBRATION_MENU STATE DEF
  // ===================================
  case CALIBRATION_MENU: {
    // State initialization code:
    if (currentState != lastState) {
      Serial.println("[SYSTEM] State changed to: CALIBRATION MENU");
      Serial.println("      Select option with Left joystick and press X to start");
      Serial.println(opts_CALIBRATION_MENU[calibrationMenuOption]);
      Serial.println("[CAL] AUTO PICKUP TUNING: WIDE/TALL pickup settings; TUNING & STORAGE: shared/test settings + calibrations");
      // PE_buttons = 0;
      int8_t current_move[7] = {(int8_t)-currentControlCMD.jsLy, 0, 0, 0, 0, 0, 0};
      requestedCustomAlgo = false;
      requestedAlgoVelocity = false;
      requestedTagsAlgo = false;
      autoTuningLaunchPending=false;
      memcpy(last_move, current_move, sizeof(last_move));
      lastState = currentState;
    }
    if (!udpControllerStable) break;
    // State running code:

    int8_t current_move[7] = {(int8_t)-currentControlCMD.jsLy, 0, 0, 0, 0, 0, 0};

    bool anyMotion = false;
    if (current_move[0] != 0)
      anyMotion = true;

    if ((memcmp(current_move, last_move, sizeof(current_move)) != 0)) {
      if (anyMotion) {
        int8_t tempMenu_option = (int8_t)calibrationMenuOption + current_move[0];
        if (tempMenu_option < 0)
          tempMenu_option = 0;
        else if (tempMenu_option > 5)
          tempMenu_option = 5;
        if (tempMenu_option != calibrationMenuOption) {
          calibrationMenuOption = tempMenu_option;
          Serial.print("[MENU] option changed to ");
          Serial.println(opts_CALIBRATION_MENU[calibrationMenuOption]);
        }
      }
      memcpy(last_move, current_move, sizeof(last_move));
    }

    // State exit code:
    if (currentControlCMD.buttons & BTN_B) {
      entered_ES = true;
      currentState = EMERGENCY_STOP;
    } else if (((currentControlCMD.buttons & BTN_L1) && (currentControlCMD.buttons & BTN_R2)) ||
               ((currentControlCMD.buttons & BTN_L2) && (currentControlCMD.buttons & BTN_R1))) {
      currentState = MODE_SELECTOR;
    } else if (PE_buttons & BTN_X) {
      if (calibrationMenuOption == 0) {
        if (cameraOffline || communicationError)
          Serial.println("[WARN] Camera unavailable; trajectory test is still available");
        else {
          requestedTagsAlgo = true;
          currentState = WAITING_ALGO_SWITCH;
        }
      } else if (calibrationMenuOption == 1) {
        if (cameraOffline || communicationError) {
          Serial.println("[WARN] Camera unavailable; trajectory test is still available");
        } else if (homographyDone) {
          requestedAlgoVelocity = true;
          currentState = WAITING_ALGO_SWITCH;
        } else {
          Serial.println("[WARN] no homography done");
        }
      } else if (calibrationMenuOption == 2) {
        currentState = TRAJECTORY_TEST;
      } else if (calibrationMenuOption == 3) {
        currentState = JOINT_MOTION_TEST;
      } else if(calibrationMenuOption==4) {
        tuningReturnState=CALIBRATION_MENU; currentState=TUNING_MENU;
      } else if(cameraOffline || communicationError) {
        Serial.println("[WARN] Camera unavailable; AUTO pickup tuning needs live parts");
      } else {
        autoTuningLaunchPending=true;currentState=WAITING_ALGO_SWITCH;
      }
    }
    break;
  }
  // ===================================
  // TRAJECTORY_TEST STATE DEF
  // ===================================
  case TRAJECTORY_TEST: {
    uint32_t now = millis();
    if ((PE_buttons & BTN_R3) && !(PE_buttons & BTN_L1)) {
      tuningMenuRequested=true; tuningReturnState=TRAJECTORY_TEST;
      Serial.println("[TUNE] Requested; finish cycle/HOME first. If paused, A resumes it");
    }
    if (tuningMenuRequested && !arm.busy()) {currentState=TUNING_MENU;break;}
    if (currentState != lastState) {
      arm.cancel();
      trajectoryTestRun = TrajectoryTestRun::Ready;
      // Placeholder configuration for admission/cancel before the first cycle.
      trajectoryTestConfig =
          TrajectoryTest::configuration(autoConfig, TrajectoryTest::minimumXmm, params, 0);
      randomSeed(micros());
      testLastStage = "";
      testLastPaused = false;
      newLatest = false;
      lastState = currentState;
      Serial.println("[TEST] A: start/resume; Y: pause after current action; L1: cancel/HOME; R3: finish/HOME then tuning menu");
      char settings[192];
      snprintf(settings, sizeof(settings),
               "[TEST] Pickup X range=%ld..%ldmm, initial Y=%.0fmm, "
               "velocity=%.0fmm/s; camera not required\n",
               TrajectoryTest::minimumXmm, TrajectoryTest::maximumXmm,
               TrajectoryTest::initialYmm, TrajectoryTest::velocityMmS);
      Serial.print(settings);
      Serial.println("[TEST] Tuning menu can fix X/yaw/bin for repeatability. Pickup uses shared fit/tilt; "
                     "q5 held during sorting");
      Serial.println("[TEST] Mapped part ID/bin (random or selected in tuning); sorting shoulder pitch=0; "
                     "q4 held through waypoint, q5 held through sorting");
      Serial.println("[TEST] After sorting/opening, return HOME before the next cycle");
      for (const Destination &destination : trajectoryTestConfig.destinations) {
        char message[128];
        snprintf(message, sizeof(message), "[TEST] Bin %u enabled=%u TCP XYZ={%.2f,%.2f,%.2f}\n",
                 unsigned(destination.binId), destination.enabled, destination.pose[0],
                 destination.pose[1], destination.pose[2]);
        Serial.print(message);
      }
    }

    // Limits and fresh Nucleo telemetry are still required for physical motion.
    if (!limits_received && now - last_limit_req_time > 1000) {
      Serial1.print("S:REQ_LIMITS\n");
      last_limit_req_time = now;
    }
    if ((PE_buttons & BTN_L1) && trajectoryTestRun != TrajectoryTestRun::ReturningHome) {
      tuningMenuRequested=false;
      if (!armConfigurationReady(trajectoryTestConfig, false)) {
        latchEmergency("Cannot return HOME: Nucleo feedback, limits or configuration unavailable");
        break;
      }
      if (arm.busy())
        arm.cancelPickup(armFeedback, now);
      else
        arm.prepare(trajectoryTestConfig, armFeedback, now, arm.clawMayHoldPart());
      trajectoryTestRun = TrajectoryTestRun::ReturningHome;
      Serial.println("[TEST] Cancelled; returning HOME; a closing/closed claw stays CLOSED");
    } else if (trajectoryTestRun != TrajectoryTestRun::ReturningHome) {
      // Stop/pause takes priority over a simultaneous A press.
      if ((PE_buttons & BTN_Y) && arm.busy() && !arm.paused()) {
        arm.requestPause(now);
        Serial.println("[TEST] Pause requested; imaginary part clock frozen");
      } else if (PE_buttons & BTN_A) {
        if (arm.paused()) {
          arm.resume(now);
          Serial.println("[TEST] Resuming sequence");
        } else if (trajectoryTestRun == TrajectoryTestRun::Ready) {
          if (!Classification::activeRouteCount(trajectoryTestConfig))
            Serial.println("[TEST] No valid mapped part IDs; edit ClassificationConfig.h");
          else if (const char *reason = armConfigurationIssue(trajectoryTestConfig, true))
            printReadinessFailure("TEST", reason, trajectoryTestConfig);
          else {
            size_t choices = Classification::activeRouteCount(trajectoryTestConfig);
            const Classification::PartRoute *route =
                Classification::activeRoute(random(0, choices), trajectoryTestConfig);
            if (activeTuning.testBin) {
              route=nullptr;
              for(size_t i=0;i<choices;++i) {
                const auto *candidate=Classification::activeRoute(i,trajectoryTestConfig);
                if(candidate && candidate->binId==activeTuning.testBin) {route=candidate;break;}
              }
            }
            if (!route) {
              Serial.println("[TEST] No valid mapped part IDs; edit ClassificationConfig.h");
              break;
            }
            trajectoryTestPartId = route->partId;
            trajectoryTestBinId = route->binId;
            trajectoryTestTallPart = activeTuning.testYaw ? activeTuning.testYaw==2 : random(0,2)!=0;
            // Arduino random() excludes its upper bound. Choose once for the
            // new cycle; HOME, pauses and resumes keep the same X and height.
            trajectoryTestXmm = activeTuning.fixedTestX ? activeTuning.testX :
                random(TrajectoryTest::minimumXmm,TrajectoryTest::maximumXmm+1);
            const AutoConfig testBase=TrajectoryTest::configuration(autoConfig,trajectoryTestXmm,
                                                                     params,trajectoryTestBinId);
            trajectoryTestProfiles=activeTuning.plans(testBase,trajectoryTestXmm,false);
            trajectoryTestConfig=trajectoryTestProfiles.config[trajectoryTestTallPart ? 1:0];
            char message[256];
            snprintf(message, sizeof(message),
                     "[TEST] Selected X=%.1f mm, pickup TCP Z=%.2f mm, approach Z=%.2f mm; "
                     "yaw=%u deg (last imaginary box); tilt=%.3f deg; part ID=%u -> bin=%u\n",
                     trajectoryTestXmm, trajectoryTestConfig.pickupZ,
                     trajectoryTestConfig.approachZ, trajectoryTestTallPart ? 90U : 0U,
                     -PickMotion::surfaceTiltDegrees(),
                     unsigned(trajectoryTestPartId), unsigned(trajectoryTestBinId));
            Serial.print(message);
            arm.prepare(trajectoryTestConfig, armFeedback, now);
            trajectoryTestRun = TrajectoryTestRun::Preparing;
            Serial.println("[TEST] Preparing HOME/open before releasing imaginary part");
          }
        }
      }
    }

    // Supervision continues during pauses and both kinds of HOME return.
    arm.tick(armFeedback, now);
    printArmProgress(now, false);
    uint32_t id = 0;
    PickOutcome result = arm.takeOutcome(id);
    if (result == PickOutcome::Fault) {
      printArmProgress(now, true);
      latchEmergency(arm.fault());
      break;
    }
    if (result == PickOutcome::HomeReady) {
      if (tuningMenuRequested) {
        trajectoryTestRun=TrajectoryTestRun::Ready; currentState=TUNING_MENU;
      } else if (trajectoryTestRun == TrajectoryTestRun::ReturningHome) {
        Serial.println("[TEST] HOME reached; returning to calibration menu");
        trajectoryTestRun = TrajectoryTestRun::Ready;
        currentState = CALIBRATION_MENU;
      } else if (trajectoryTestRun == TrajectoryTestRun::Preparing) {
        startImaginaryPart(now);
      } else if (trajectoryTestRun == TrajectoryTestRun::FinishingHome) {
        trajectoryTestRun = TrajectoryTestRun::Ready;
        Serial.println("[TEST] Complete at HOME; A starts a new test");
      }
    } else if (result == PickOutcome::Sorted) {
      // OPEN has completed. Reuse supervised HOME; stay busy until fresh
      // feedback confirms arrival/settling. A cannot release another part yet.
      if (!armConfigurationReady(trajectoryTestConfig, false)) {
        latchEmergency("Cannot finish at HOME: Nucleo feedback, limits or configuration unavailable");
        break;
      }
      if (!arm.prepare(trajectoryTestConfig, armFeedback, now)) {
        latchEmergency("Cannot start final HOME movement");
        break;
      }
      trajectoryTestRun = TrajectoryTestRun::FinishingHome;
      Serial.println("[TEST] Part sorted; returning HOME");
    } else if (result == PickOutcome::Missed || result == PickOutcome::Rejected) {
      if (result == PickOutcome::Rejected) {
        printPlanRejection();
        if (!arm.prepare(trajectoryTestConfig, armFeedback, now)) {
          latchEmergency("Cannot return HOME after rejected test pickup");
          break;
        }
        trajectoryTestRun = TrajectoryTestRun::FinishingHome;
        Serial.println("[TEST] Target rejected; returning HOME; see [PLAN] diagnostics");
      } else {
        const bool exit = trajectoryTestRun == TrajectoryTestRun::ReturningHome;
        trajectoryTestRun = TrajectoryTestRun::Ready;
        Serial.println(arm.clawMayHoldPart()
                           ? "[TEST] Recovered at HOME, claw CLOSED. Secure/remove any held part; A opens and retries"
                           : "[TEST] Recovered at HOME; A retries");
        if (tuningMenuRequested) currentState=TUNING_MENU;
        else if (exit) currentState = CALIBRATION_MENU;
      }
    }
    // Log the stage after outcome dispatch, avoiding a transient IDLE between
    // OPEN and the final HOME movement.
    printArmProgress(now, false);
    const char *stage = arm.phaseName();
    if (strcmp(stage, testLastStage) || arm.paused() != testLastPaused) {
      Serial.print("[TEST] Stage: ");
      Serial.print(stage);
      Serial.println(arm.paused() ? " [PAUSED - press A]" : "");
      testLastStage = stage;
      testLastPaused = arm.paused();
    }
    break;
  }
  // ===================================
  // WAITING_ALGO_SWITCH STATE DEF
  // ===================================
  case WAITING_ALGO_SWITCH: {
    serviceAlgorithmSelection();
    break;
  }

  // ===================================
  // VELOCITY_CALIBRATION STATE DEF
  // ===================================
  case VELOCITY_CALIBRATION: {
    serviceVelocityCalibration(millis(), PE_buttons);
    break;
  }

  // ===================================
  // CAMERA_CALIBRATION STATE DEF
  // ===================================
  case CAMERA_CALIBRATION: { serviceHomographyCalibration(millis(),PE_buttons); break; }
  }
}
