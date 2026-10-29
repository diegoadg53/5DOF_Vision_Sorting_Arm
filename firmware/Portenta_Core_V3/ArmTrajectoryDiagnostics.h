#pragma once
#include "ArmTrajectory.h"
#include <stdio.h>

inline void printClassificationPlan(Print &out,const ClassificationPlanReport &r) {
  char b[320];
  snprintf(b,sizeof(b),"[CLASS PLAN] track=%lu task=%s shoulders=%s q5=%s XYZerror=%.3fmm rotationError=%.3fdeg\n",
    (unsigned long)r.trackId,ik_task_name(r.ik.task),r.fixedShoulders ? "pitch0/bin-roll":"legacy",
    r.heldPickupQ5 ? "pickup-held":"free",r.ik.positionErrorMm,r.ik.fullRotationErrorRad*180/PI);
  out.print(b);
  snprintf(b,sizeof(b),"[CLASS PLAN] requested TCP=%.2f,%.2f,%.2f RPY=%.2f,%.2f,%.2f; planned TCP=%.2f,%.2f,%.2f RPY=%.2f,%.2f,%.2f\n",
    r.requested[0],r.requested[1],r.requested[2],r.requested[3],r.requested[4],r.requested[5],
    r.planned[0],r.planned[1],r.planned[2],r.planned[3],r.planned[4],r.planned[5]);out.print(b);
  snprintf(b,sizeof(b),"[CLASS PLAN] joints[pitch,roll,q3,q4,q5]=%.2f,%.2f,%.2f,%.2f,%.2fdeg\n",
    r.joints[0],r.joints[1],r.joints[2],r.joints[3],r.joints[4]);out.print(b);
}

// Print only captured event-time data, never the live recovery target/config.
inline void printArmDiagnostic(Print &out, const ArmDiagnosticReport &d) {
  char b[384];
  const ArmProgress &p = d.progress;
  const char *profile = d.tallPickup ? "TALL" : "WIDE";
  if (d.event == ArmDiagnosticEvent::TimedPickupComplete) {
    snprintf(b, sizeof(b),
             "[PICK TIMING] %s_TIMED_COMPLETE track=%lu remainingArrival=%.1fms "
             "closeElapsed=%lums requiredClose=%lums postClose=%lums "
             "grippingPoseCheck=not-required TCPerror=%.2fmm feedbackAge=%lums\n",
             profile, (unsigned long)d.trackId, d.remainingArrivalMs, (unsigned long)d.closeElapsedMs,
             (unsigned long)(d.closingDurationMs + d.postCloseMs), (unsigned long)d.postCloseMs,
             p.positionErrorMm, (unsigned long)p.feedbackAgeMs);
    out.print(b);
    return;
  }
  if (d.event != ArmDiagnosticEvent::Missed) {
    const char *name = d.event == ArmDiagnosticEvent::SafetyConfirmed ? "SAFETY_CONFIRMED" :
                       d.event == ArmDiagnosticEvent::GrippingConfirmed ? "GRIPPING_CONFIRMED" :
                       d.event == ArmDiagnosticEvent::DescentCommand ? "DESCENT_COMMAND" : "CLOSE_COMMAND";
    snprintf(b, sizeof(b),
             "[PICK TIMING] %s track=%lu remainingArrival=%.1fms targetElapsed=%lums "
             "stable=%lu/%lums deadlineMargin=%+.1fms tickGap=%lums feedbackAge=%lums\n",
             name, (unsigned long)d.trackId, d.remainingArrivalMs, (unsigned long)p.elapsedMs,
             (unsigned long)p.stableMs, (unsigned long)d.settleMs,
             d.remainingArrivalMs - d.thresholdMs, (unsigned long)p.tickGapMs,
             (unsigned long)p.feedbackAgeMs);
    out.print(b);
    if (d.event == ArmDiagnosticEvent::DescentCommand && d.timedPickup) {
      snprintf(b, sizeof(b),
               "[PICK POLICY] %s timed completion; gripping pose/settling not required; "
               "transport after actual close command + %lums closing + %lums post-close\n",
               profile, (unsigned long)d.closingDurationMs, (unsigned long)d.postCloseMs);
      out.print(b);
    }
    return;
  }
  snprintf(b, sizeof(b),
           "[PICK MISSED] reason=%s track=%lu stage=%s target=%s remainingArrival=%.1fms; "
           "returning HOME; claw=%s\n", d.reason, (unsigned long)d.trackId, d.stage,
           d.targetName, d.remainingArrivalMs, d.closingStarted ? "CLOSED" : "OPEN");
  out.print(b);
  if (d.hasDeadline) {
    snprintf(b, sizeof(b),
             "[PICK DEADLINE] requirement=%s thresholdRemaining=%.1fms "
             "margin=%+.1fms allowedLate=%.1fms\n", d.requirement, d.thresholdMs,
             d.remainingArrivalMs - d.thresholdMs, d.allowedLateMs);
    out.print(b);
  }
  unsigned mask = 0;
  for (int i=0; i<5; ++i)
    if (!isfinite(p.error[i]) || fabsf(p.error[i]) > p.jointToleranceDeg[i]) mask |= 1U << i;
  const float activeTolerance = p.settlingStarted ? p.positionHoldToleranceMm : p.positionEntryToleranceMm;
  const bool tcpOK = isfinite(p.positionErrorMm) && p.positionErrorMm <= activeTolerance;
  snprintf(b, sizeof(b),
           "[PICK CHECK] feedbackValid=%u feedbackAge=%lums jointsOK=%u TCPok=%u "
           "TCPerror=%.2f/%.2fmm stable=%lu/%lums targetElapsed=%lums minimumMove=%lums "
           "tickGap=%lums maxTickGap=%lums feedbackInterval=%lums\n",
           p.feedbackValid, (unsigned long)p.feedbackAgeMs, p.feedbackValid && !mask,
           p.feedbackValid && tcpOK, p.positionErrorMm, activeTolerance,
           (unsigned long)p.stableMs, (unsigned long)d.settleMs, (unsigned long)p.elapsedMs,
           (unsigned long)d.minimumMoveMs, (unsigned long)p.tickGapMs,
           (unsigned long)p.maximumTickGapMs, (unsigned long)p.feedbackIntervalMs);
  out.print(b);
  snprintf(b, sizeof(b),
           "[PICK JOINTS] order=pitch,roll,j3,q4,q5 target=%.2f,%.2f,%.2f,%.2f,%.2f "
           "error(actual-target)=%.2f,%.2f,%.2f,%.2f,%.2f "
           "tolerance=%.2f,%.2f,%.2f,%.2f,%.2fdeg outsideMask=0x%02X "
           "q4entry/hold=%.2f/%.2fdeg holdActive=%u\n",
           p.target[0], p.target[1], p.target[2], p.target[3], p.target[4],
           p.error[0], p.error[1], p.error[2], p.error[3], p.error[4],
           p.jointToleranceDeg[0], p.jointToleranceDeg[1], p.jointToleranceDeg[2],
           p.jointToleranceDeg[3], p.jointToleranceDeg[4], mask,
           p.jointEntryToleranceDeg[3], p.jointHoldToleranceDeg[3], p.jointHoldActive);
  out.print(b);
  snprintf(b, sizeof(b),
           "[PICK TUNING] approachMargin=%lums descentLead=%lums closingLead=%lums "
           "closingDuration=%lums pickupBudget=%lums closeStarted=%u closeElapsed=%lums\n",
           (unsigned long)d.marginMs, (unsigned long)d.descentLeadMs,
           (unsigned long)d.closingLeadMs, (unsigned long)d.closingDurationMs,
           (unsigned long)d.budgetMs, d.closingStarted, (unsigned long)d.closeElapsedMs);
  out.print(b);
}
