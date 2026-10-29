#include "FakeMbedUdp.h"
#include "PS3DataPacket.h"
#include "VisionPacket.h"
#include "NonBlockingVisionUdp.h"
#include "ControllerLinkDiagnostics.h"
#include "CameraAlgorithmSelection.h"
#include <stdio.h>
#include <new>

FakeWifi WiFi;
#ifdef REPRODUCE_OLD_DRAIN
class TestVisionUdp : public WiFiUDP {
public:
  FakeSocket &transport() { return _socket; }
  bool sendTo(IPAddress peer, uint16_t port, const uint8_t *data, size_t size) {
    beginPacket(peer, port); write(data, size); return endPacket() == int(size);
  }
};
#else
class TestVisionUdp : public NonBlockingVisionUdp {
public:
  FakeSocket &transport() { return socket_; }
};
#endif

// MbedUDP allocates its receive buffers. Use a bounded fixture heap; no OS calls.
static uint8_t heap[32768];
static size_t heapAt = 0;
void *operator new(size_t size) {
  size = (size + 7) & ~size_t(7);
  void *p = heap + heapAt;
  heapAt += size;
  return p;
}
void *operator new[](size_t size) {
  return ::operator new(size);
}
void operator delete(void *) noexcept {}
void operator delete[](void *) noexcept {}
void operator delete(void *, size_t) noexcept {}
void operator delete[](void *, size_t) noexcept {}
extern "C" {
void *__dso_handle = nullptr;
}
extern "C" void _fini() {}

struct TestResult {
  uint32_t checks, failures, firstFailure;
};
extern "C" {
volatile TestResult test_result = {};
}
static void check(bool value, uint32_t code) {
  ++test_result.checks;
  if (!value) {
    ++test_result.failures;
    if (!test_result.firstFailure)
      test_result.firstFailure = code;
  }
}
FakeSerial Serial;
uint32_t testTime = 1000;
uint32_t millis() {
  return testTime;
}
WiFiUDP udpController;
TestVisionUdp cameraUDP;
IPAddress esp32IP;
bool esp32IP_known = false, udpControllerStable = false;
uint32_t lastSeqReceived = 0, lastTimeReceived = 0, visionTimestampMs = 0;
const uint16_t cameraUDPport = 4220;
const uint32_t TIMEOUT_FAILSAFE_MS = 200;
constexpr uint16_t BTN_B = 1;
ControllerLinkDiagnostics controllerLink;
PS3ControlPacket currentControlCommand;
uint16_t controllerPressedEdges;
bool controllerEmergencyReceived, controllerTimedOutThisLoop, newControlCMD;
criticalMessage pendingESPmsg;
uint8_t lastAckType = 0, totalObjects = 0;
bool messageAck = false, newLatest = false, cameraOffline = true, algoSwitched = false,
     invalidAlgo = false;
objectVision latestReceived[10];
CameraAlgorithmSelection cameraAlgorithm;

// Extracted by the runner from the actual sketch, not reimplemented here.
#include "AppUdpState.inc"
static State currentState = MODE_SELECTOR;
static uint8_t menu_option2 = 0;
#include "AppUdpFunctions.inc"

static IPAddress peerA() {
  return {192, 168, 4, 2};
}
static IPAddress peerB() {
  return {192, 168, 4, 3};
}
static void reset() {
  currentState = MODE_SELECTOR;
  menu_option2 = 0;
  new (&udpController) WiFiUDP;
  new (&cameraUDP) TestVisionUdp;
  udpController.begin(4210);
  cameraUDP.begin(4220);
  esp32IP_known = udpControllerStable = false;
  lastSeqReceived = lastTimeReceived = 0;
  messageAck = newLatest = algoSwitched = invalidAlgo = false;
  cameraOffline = true;
  cameraAlgorithm.invalidate();
  pendingESPmsg = criticalMessage{};
  controllerLink = ControllerLinkDiagnostics{};
  currentControlCommand = PS3ControlPacket{};
  controllerPressedEdges = 0;
  controllerEmergencyReceived = controllerTimedOutThisLoop = newControlCMD = false;
  testTime = 1000;
}

static void queueControl(IPAddress source, uint32_t seq, bool corrupt = false, uint16_t buttons = 2) {
  PS3ControlPacket p{};
  p.seq = seq;
  p.buttons = buttons;
  p.lx = 100;
  p.checksum = computeChecksum((const uint8_t *)&p, sizeof(p) - 1);
  if (corrupt)
    p.checksum ^= 0xff;
  udpController._socket.enqueue(&p, sizeof(p), source);
}
static void calibrationStillRecordsRealControllerTimeouts() {
  const State nonMotion[]={CAMERA_CALIBRATION,VELOCITY_CALIBRATION,MODE_SELECTOR,
                          AUTO_MENU,CALIBRATION_MENU,WAITING_ALGO_SWITCH,AUTO_MODE};
  for (State state:nonMotion) {
    reset(); currentState=state; menu_option2=1;
    queueControl(peerA(),1,false,0); serviceControllerPackets();
    cameraAlgorithm.begin(16); cameraAlgorithm.switched(); cameraAlgorithm.acknowledge();
    testTime+=TIMEOUT_FAILSAFE_MS+1; serviceControllerPackets();
    check(!udpControllerStable && controllerTimedOutThisLoop &&
          !controllerLossRequiresStop() && controllerLink.awaitingRecovery,1600);
    check(cameraAlgorithm.matches(16),1603);
    // Vision packets remain usable during the controller gap.
    cameraOffline=false;
    VisionPacket p{}; p.type=DATA_OBJECTS; p.vision.amountoObjects=0;
    cameraUDP.transport().enqueue(&p,sizeof(p),peerA()); manageVisionPort();
    check(newLatest && !udpControllerStable && !cameraOffline,1601);
    queueControl(peerA(),2,false,BTN_B); serviceControllerPackets();
    check(udpControllerStable && controllerTimedOutThisLoop &&
          controllerEmergencyReceived && controllerLink.recoveryPending,1602);
  }
}
static VisionPacket event(uint8_t type, uint8_t id = 7) {
  VisionPacket p{};
  p.type = type;
  p.msgID = id;
  return p;
}
static void vendorReadSemantics() {
  reset();
  WiFiUDP vendor;
  vendor.begin(4221);
  VisionPacket p = event(EVT_CAM_ONLINE);
  vendor._socket.enqueue(&p, sizeof(p), peerA());
  check(vendor.parsePacket() == sizeof(p), 101);
  VisionPacket read{};
  check(vendor.read((uint8_t *)&read, sizeof(read)) == sizeof(read), 102);
  check(vendor.available() == sizeof(p), 103); // Actual installed Mbed behaviour.
}
static void cameraBeforeController() {
  reset();
  VisionPacket p = event(EVT_CAM_ONLINE);
  cameraUDP.transport().enqueue(&p, sizeof(p), peerA());
  manageVisionPort();
  check(!esp32IP_known && cameraUDP.transport().sent == 0, 201);
  queueControl(peerA(), 1);
  PS3ControlPacket controller{};
  check(processUDPPacket(controller), 202);
  check(esp32IP_known && esp32IP == peerA() && controller.buttons == 2, 203);
  cameraUDP.transport().enqueue(&p, sizeof(p), peerA());
  manageVisionPort();
  check(!cameraOffline && cameraUDP.transport().sent == 1, 204);
  VisionPacket ack{};
  memcpy(&ack, cameraUDP.transport().lastSent, sizeof(ack));
  check(ack.type == ACK_OK && ack.msgID == 7 && ack.ack.typeAck == EVT_CAM_ONLINE, 205);
}
static void sameLoopStartup() {
  reset();
  queueControl(peerA(), 10);
  PS3ControlPacket controller{};
  check(processUDPPacket(controller), 301);
  // Peer must already be known, before WAITING_CONNECTION state code runs.
  check(esp32IP_known, 302);
  VisionPacket p = event(EVT_CAM_OFFLINE, 9);
  cameraUDP.transport().enqueue(&p, sizeof(p), peerA());
  manageVisionPort();
  check(cameraUDP.transport().sent == 1 && cameraOffline, 303);
}
static void badControllerDoesNotRegisterPeer() {
  reset();
  queueControl(peerA(), 1, true);
  PS3ControlPacket p{};
  check(!processUDPPacket(p) && !esp32IP_known && lastTimeReceived == 0, 401);
  uint8_t wrong[4]{};
  udpController._socket.enqueue(wrong, 4, peerA());
  check(!processUDPPacket(p) && !esp32IP_known, 402);
}
static void wrongVisionPacketsBounded() {
  reset();
  queueControl(peerA(), 1);
  PS3ControlPacket p{};
  processUDPPacket(p);
  VisionPacket v = event(EVT_CAM_ONLINE);
  cameraUDP.transport().enqueue(&v, sizeof(v), peerB());
  uint8_t wrong[5]{};
  cameraUDP.transport().enqueue(wrong, 5, peerA());
  cameraUDP.transport().enqueue(&v, sizeof(v), peerA());
  manageVisionPort();
  check(cameraUDP.transport().sent == 1 && !cameraOffline, 501);
  reset();
  for (int i = 0; i < 16; ++i)
    cameraUDP.transport().enqueue(wrong, 5, peerA());
  manageVisionPort();
  check(cameraUDP.transport().receiveCalls == 8 && cameraUDP.transport().head == 8, 502);
}
static void controllerPeerAndReconnection() {
  reset();
  PS3ControlPacket p{};
  queueControl(peerA(), 100);
  processUDPPacket(p);
  udpControllerStable = true;
  queueControl(peerB(), 101);
  check(!processUDPPacket(p) && esp32IP == peerA() && lastSeqReceived == 100, 601);
  queueControl(peerA(), 99);
  check(!processUDPPacket(p) && lastSeqReceived == 100, 602);
  udpControllerStable = false;
  testTime += 1000;
  queueControl(peerB(), 1);
  check(processUDPPacket(p) && esp32IP == peerB() && lastSeqReceived == 1 &&
            lastTimeReceived == testTime,
        603);
  VisionPacket v = event(EVT_CAM_ONLINE);
  cameraUDP.transport().enqueue(&v, sizeof(v), peerB());
  manageVisionPort();
  check(!cameraOffline && cameraUDP.transport().lastDestination.ip == peerB(), 604);
}
static void ackMatchingAndFrameValidation() {
  reset();
  queueControl(peerA(), 1);
  PS3ControlPacket p{};
  processUDPPacket(p);
  pendingESPmsg.pending = true;
  pendingESPmsg.pkt.type = CMD_END_STREAM;
  pendingESPmsg.pkt.msgID = 11;
  VisionPacket ack = event(ACK_OK, 11);
  ack.ack.typeAck = CMD_START_STREAM;
  cameraUDP.transport().enqueue(&ack, sizeof(ack), peerA());
  manageVisionPort();
  check(pendingESPmsg.pending && !messageAck, 701);
  ack.ack.typeAck = CMD_END_STREAM;
  cameraUDP.transport().enqueue(&ack, sizeof(ack), peerA());
  manageVisionPort();
  check(!pendingESPmsg.pending && messageAck && lastAckType == CMD_END_STREAM, 702);
  VisionPacket frame = event(DATA_OBJECTS);
  frame.vision.amountoObjects = 11;
  cameraUDP.transport().enqueue(&frame, sizeof(frame), peerA());
  manageVisionPort();
  check(!newLatest, 703);
  frame.vision.amountoObjects = 1;
  frame.vision.objList[0].xCenter = 332;
  cameraUDP.transport().enqueue(&frame, sizeof(frame), peerA());
  manageVisionPort();
  check(newLatest && totalObjects == 1 && latestReceived[0].xCenter == 332, 704);
}
static void rejectedPacketAheadOfFreshControl() {
  reset();
  queueControl(peerA(), 100, false, 0);
  serviceControllerPackets();
  testTime += 201;
  queueControl(peerA(), 99);
  queueControl(peerA(), 101, true);
  queueControl(peerA(), 102, false, 0);
  serviceControllerPackets();
  check(udpControllerStable && !controllerTimedOutThisLoop && lastSeqReceived == 102, 801);
  check(controllerLink.staleSequence == 1 && controllerLink.badChecksum == 1 &&
            controllerLink.accepted == 2 && controllerLink.lastAcceptedGapMs == 201, 802);
  check(controllerLink.maxPollGapMs == 201, 803);
}
static void shortPressAndEmergencyArePreserved() {
  reset();
  queueControl(peerA(), 1, false, 0);
  queueControl(peerA(), 2, false, BTN_B | 2);
  queueControl(peerA(), 3, false, 0);
  serviceControllerPackets();
  check(currentControlCommand.buttons == 0 && controllerPressedEdges == (BTN_B | 2), 901);
  check(controllerEmergencyReceived && newControlCMD && udpControllerStable, 902);
}
static void realTimeoutStillLatchesInThisIteration() {
  reset();
  queueControl(peerA(), 1);
  serviceControllerPackets();
  testTime += 201;
  serviceControllerPackets();
  check(!udpControllerStable && controllerTimedOutThisLoop && lastTimeReceived == 1000, 1001);
  check(controllerLink.timeoutAgeMs == 201 && controllerLink.timeoutSequence == 1, 1003);
  queueControl(peerA(), 2);
  serviceControllerPackets();
  check(udpControllerStable && controllerTimedOutThisLoop, 1002);
  check(controllerLink.timeoutAgeMs == 201 && controllerLink.timeoutSequence == 1, 1004);
  // The global supervisor uses the retained flag, so recovery cannot cancel
  // the emergency request before the main switch handles it.
}
static void boundedReceiveWorkAndSequenceWrap() {
  reset();
  for (int i = 0; i < 12; ++i)
    queueControl(peerA(), i + 1, false, 0);
  serviceControllerPackets();
  check(lastSeqReceived == 8 && udpController._socket.head == 8, 1101);
  serviceControllerPackets();
  check(lastSeqReceived == 12 && udpController._socket.head == 12, 1102);
  reset();
  queueControl(peerA(), 0xfffffffe, false, 0);
  serviceControllerPackets();
  queueControl(peerA(), 1, false, 0);
  testTime += 20;
  serviceControllerPackets();
  check(lastSeqReceived == 1 && controllerLink.staleSequence == 0 && udpControllerStable, 1103);
}
static void nonblockingVisionTxAndRx() {
  reset();
  queueControl(peerA(), 1);
  serviceControllerPackets();
  VisionPacket p = event(EVT_CAM_ONLINE);
  sendVisionPacket(p);
  check(!cameraUDP.transport().lastSendBlocking && cameraUDP.transport().timeout == 0 &&
            cameraUDP.transport().boundPort == 4220 &&
            cameraUDP.transport().lastDestination.port == 4220, 1201);
  cameraUDP.transport().sendError = NSAPI_ERROR_WOULD_BLOCK;
  uint32_t before = testTime;
  sendVisionPacket(p);
  check(testTime == before && controllerLink.visionTxFailures == 1, 1202);
  serviceControllerPackets();
  check(udpControllerStable && !controllerTimedOutThisLoop, 1203);
  cameraUDP.transport().enqueue(&p, sizeof(p), peerA());
  check(cameraUDP.parsePacket() == sizeof(p), 1204);
  VisionPacket received{};
  check(cameraUDP.read((uint8_t *)&received, sizeof(received)) == sizeof(p) &&
            cameraUDP.available() == 0 && memcmp(&received, &p, sizeof(p)) == 0, 1205);
  check(cameraUDP.parsePacket() == 0 && cameraUDP.available() == 0, 1206);
}

static void recoveryDiagnosticsDoNotClearEmergency() {
  reset();
  queueControl(peerA(), 100, false, 0);
  serviceControllerPackets();
  testTime += 201;
  serviceControllerPackets();
  check(controllerLink.awaitingRecovery && !controllerLink.recoveryPending, 1301);
  testTime += 39;
  queueControl(peerA(), 112, false, 0);
  queueControl(peerA(), 113, false, 0);
  serviceControllerPackets();
  check(controllerTimedOutThisLoop && udpControllerStable, 1302);
  check(controllerLink.recoveryPending && !controllerLink.awaitingRecovery &&
            controllerLink.recoveryGapMs == 240 && controllerLink.recoverySequence == 112 &&
            controllerLink.recoverySequenceStep == 12 &&
            controllerLink.recoverySequenceComparable, 1303);
  check(controllerLink.missingSequences == 11 && controllerLink.maxAcceptedGapMs == 240, 1304);
  check(controllerLink.timeoutSequence == 100 && controllerLink.timeoutAgeMs == 201, 1305);
  reportControllerRecovery();
  check(!controllerLink.recoveryPending && controllerTimedOutThisLoop, 1306);
  // No sequence skips: sender pause or a delayed datagram, not proof of RF loss.
  controllerTimedOutThisLoop = false;
  testTime += 201;
  serviceControllerPackets();
  testTime += 49;
  queueControl(peerA(), 114, false, 0);
  serviceControllerPackets();
  check(controllerLink.recoverySequenceStep == 1 && controllerLink.recoveryGapMs == 250 &&
            controllerLink.missingSequences == 11 && controllerTimedOutThisLoop, 1307);
}

static void recoverySequenceResetPeerAndWrap() {
  reset();
  queueControl(peerA(), 100, false, 0);
  serviceControllerPackets();
  testTime += 201;
  serviceControllerPackets();
  queueControl(peerA(), 1, false, 0); // ESP32 restarted its counter.
  serviceControllerPackets();
  check(controllerLink.recoveryPending && !controllerLink.recoverySequenceComparable &&
            controllerLink.recoverySequenceStep == 0 && controllerLink.missingSequences == 0,
        1401);
  reportControllerRecovery();
  controllerTimedOutThisLoop = false;
  testTime += 201;
  serviceControllerPackets();
  queueControl(peerB(), 50, false, 0);
  serviceControllerPackets();
  check(esp32IP == peerB() && !controllerLink.recoverySequenceComparable &&
            controllerLink.missingSequences == 0, 1402);

  reset();
  testTime = 0xfffffff0;
  queueControl(peerA(), 0xfffffffe, false, 0);
  serviceControllerPackets();
  testTime += 201;
  serviceControllerPackets();
  queueControl(peerA(), 1, false, 0);
  serviceControllerPackets();
  check(controllerLink.recoveryGapMs == 201 && controllerLink.recoverySequenceStep == 3 &&
            controllerLink.recoverySequenceComparable && controllerLink.missingSequences == 2 &&
            controllerTimedOutThisLoop, 1403);
}

static void algorithmCacheFollowsValidatedCameraTraffic() {
  reset();
  queueControl(peerA(), 1);
  serviceControllerPackets();
  cameraOffline = false;
  cameraAlgorithm.begin(131);
  pendingESPmsg.pending = true;
  pendingESPmsg.pkt.type = CMD_SWITCH_ALGO;
  pendingESPmsg.pkt.msgID = 41;
  VisionPacket ack = event(ACK_OK, 40);
  ack.ack.typeAck = CMD_SWITCH_ALGO;
  VisionPacket switched = event(EVT_SWITCHED_ALGO, 55);
  cameraUDP.transport().enqueue(&switched, sizeof(switched), peerB());
  cameraUDP.transport().enqueue(&ack, sizeof(ack), peerA());
  manageVisionPort();
  check(!cameraAlgorithm.matches(131) && pendingESPmsg.pending, 1500);
  ack.msgID = 41;
  cameraUDP.transport().enqueue(&ack, sizeof(ack), peerA());
  manageVisionPort();
  check(!cameraAlgorithm.matches(131) && !pendingESPmsg.pending, 1501);
  cameraUDP.transport().enqueue(&switched, sizeof(switched), peerA());
  manageVisionPort();
  check(cameraAlgorithm.matches(131), 1502);
  cameraUDP.transport().enqueue(&switched, sizeof(switched), peerA());
  manageVisionPort();
  check(cameraAlgorithm.matches(131), 1503); // Duplicate completion is harmless.
  VisionPacket offline = event(EVT_CAM_OFFLINE, 56);
  cameraUDP.transport().enqueue(&offline, sizeof(offline), peerA());
  manageVisionPort();
  check(cameraOffline && !cameraAlgorithm.matches(131), 1504);
  cameraAlgorithm.switched(); cameraAlgorithm.acknowledge();
  check(!cameraAlgorithm.matches(131), 1505);
  cameraAlgorithm.begin(16); cameraAlgorithm.switched(); cameraAlgorithm.acknowledge();
  VisionPacket online = event(EVT_CAM_ONLINE, 57);
  cameraUDP.transport().enqueue(&online, sizeof(online), peerA());
  manageVisionPort();
  check(!cameraOffline && !cameraAlgorithm.matches(16), 1506);
  cameraAlgorithm.begin(131); cameraAlgorithm.switched(); cameraAlgorithm.acknowledge();
  currentState=MANUAL_0; // Motion-mode timeouts still invalidate the algorithm cache.
  testTime += TIMEOUT_FAILSAFE_MS+1;
  serviceControllerPackets();
  check(!cameraAlgorithm.matches(131), 1507);
  cameraAlgorithm.begin(131); cameraAlgorithm.switched(); cameraAlgorithm.acknowledge();
  queueControl(peerB(), 1);
  serviceControllerPackets();
  check(esp32IP == peerB() && !cameraAlgorithm.matches(131), 1508);
}

static void algorithmSwitchSurvivesNonMotionControllerGap() {
  reset(); currentState=WAITING_ALGO_SWITCH;
  queueControl(peerA(),1,false,0); serviceControllerPackets();
  cameraAlgorithm.begin(16);
  pendingESPmsg.pending=true; pendingESPmsg.pkt.type=CMD_SWITCH_ALGO;
  pendingESPmsg.pkt.msgID=41;
  testTime+=TIMEOUT_FAILSAFE_MS+1; serviceControllerPackets();
  check(!udpControllerStable && pendingESPmsg.pending,1610);
  VisionPacket ack=event(ACK_OK,41); ack.ack.typeAck=CMD_SWITCH_ALGO;
  VisionPacket switched=event(EVT_SWITCHED_ALGO,55);
  cameraUDP.transport().enqueue(&ack,sizeof(ack),peerA());
  cameraUDP.transport().enqueue(&switched,sizeof(switched),peerA());
  manageVisionPort();
  check(cameraAlgorithm.matches(16) && !pendingESPmsg.pending && !udpControllerStable,1611);
}

extern "C" __attribute__((noinline, used)) void test_done() {
  for (;;)
    asm volatile("nop");
}
extern "C" void test_main() {
  vendorReadSemantics();
  cameraBeforeController();
  sameLoopStartup();
  badControllerDoesNotRegisterPeer();
  wrongVisionPacketsBounded();
  controllerPeerAndReconnection();
  ackMatchingAndFrameValidation();
  rejectedPacketAheadOfFreshControl();
  shortPressAndEmergencyArePreserved();
  realTimeoutStillLatchesInThisIteration();
  boundedReceiveWorkAndSequenceWrap();
  nonblockingVisionTxAndRx();
  recoveryDiagnosticsDoNotClearEmergency();
  recoverySequenceResetPeerAndWrap();
  algorithmCacheFollowsValidatedCameraTraffic();
  calibrationStillRecordsRealControllerTimeouts();
  algorithmSwitchSurvivesNonMotionControllerGap();
  test_done();
}
