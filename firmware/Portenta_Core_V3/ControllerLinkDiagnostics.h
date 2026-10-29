#pragma once
#include <stdint.h>

struct ControllerLinkDiagnostics {
  uint32_t received = 0, accepted = 0, badSize = 0, badRead = 0, badChecksum = 0;
  uint32_t wrongPeer = 0, staleSequence = 0, socketErrors = 0, visionTxFailures = 0;
  uint32_t lastPollMs = 0, lastPollGapMs = 0, maxPollGapMs = 0, maxVisionWorkMs = 0;
  uint32_t lastAcceptedGapMs = 0, maxAcceptedGapMs = 0;
  uint32_t timeoutAgeMs = 0, timeoutPollGapMs = 0, timeoutSequence = 0;
  uint32_t missingSequences = 0;
  uint32_t recoveryGapMs = 0, recoverySequence = 0, recoverySequenceStep = 0;
  int lastPacketSize = 0;
  bool polled = false;
  bool awaitingRecovery = false, recoveryPending = false, recoverySequenceComparable = false;
};
