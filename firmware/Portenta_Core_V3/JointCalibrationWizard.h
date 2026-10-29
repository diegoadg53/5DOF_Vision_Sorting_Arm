#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "JointCalibrationProtocol.h"

class JointCalibrationWizard {
public:
  using Sender = bool (*)(const char *);
  using Logger = void (*)(const char *);
  bool active = false, pending = false;
  unsigned phase = JC_IDLE, mask = 0, reason = 0, axis = 0;
  int direction = 1;
  uint32_t generation = 0;
  unsigned source = JC_DEFAULTS;
  bool statusReceived = false;
  float offsets[5]{};
  const char *failure = nullptr;

  void bind(Sender sender, Logger logger) { send_ = sender; log_ = logger; }
  void begin(uint32_t now, uint32_t session) {
    Sender s = send_; Logger l = log_; *this = JointCalibrationWizard{}; bind(s, l);
    active = true; session_ = session ? session : 1;
    lastStatus_ = lastPing_ = now; command("BEGIN", nullptr, now);
  }
  void abandon() { active = pending = false; }
  bool committing() const { return pending && !strcmp(verb_, "COMMIT"); }
  bool command(const char *verb, const char *args, uint32_t now, bool urgent = false) {
    if (!active || (pending && !urgent)) return false;
    snprintf(verb_, sizeof(verb_), "%s", verb);
    ++request_; if (!request_) ++request_;
    int n = snprintf(command_, sizeof(command_), "J:%lu,%lu,%s%s\n",
      (unsigned long)session_, (unsigned long)request_, verb, args ? args : "");
    if (n <= 0 || n >= int(sizeof(command_))) { failure = "Calibration command too long"; return false; }
    pending = true; attempts_ = 1; sent_ = now;
    if (!send_(command_)) failure = "Calibration UART write failed";
    return !failure;
  }
  void tick(uint32_t now) {
    if (!active || failure) return;
    if (!committing() && uint32_t(now - lastPing_) >= 100) {
      char line[64]; lastPing_ = now;
      snprintf(line, sizeof(line), "J:%lu,0,PING\n", (unsigned long)session_);
      if (!send_(line)) { failure = "Calibration keepalive write failed"; return; }
    }
    if (pending && uint32_t(now - sent_) >= (committing() ? 10000U : 300U)) {
      if (++attempts_ > (committing() ? 3U : 5U)) {
        failure = "Calibration acknowledgement timeout"; return;
      }
      sent_ = now;
      if (!send_(command_)) failure = "Calibration retry write failed";
    }
    if (!committing() && uint32_t(now - lastStatus_) > 2000)
      failure = "Nucleo calibration status lost";
  }
  void receive(const char *line, uint32_t now) {
    unsigned long session, id; unsigned p, m, r; int end = 0;
    if (sscanf(line, "J:%lu,%lu,%u,%u,%u%n", &session, &id, &p, &m, &r, &end) == 5 &&
        !line[end] && p <= JC_FAULT && m <= 7) {
      if (!active || session != session_ || (id && id < request_)) return;
      lastStatus_ = now;
      bool ack = pending && id == request_;
      bool save = ack && !strcmp(verb_, "SAVE");
      // COMMIT's intermediate SAVING acknowledgement is not completion.
      // Keep the UART quiet until read-back succeeds or a final error arrives.
      if ((ack && !(committing() && p == JC_SAVING && r == JC_OK)) ||
          (committing() && p == JC_SAVED)) pending = false;
      bool changed = phase != p || reason != r || mask != m;
      phase = p; mask = m; reason = r;
      if (changed) {
        char text[160];
        snprintf(text, sizeof(text), "[JOINT CAL] %s; captured[P,R,E]=%u,%u,%u reason=%u (%s)\n",
                 phaseName(p), unsigned(!!(m&1)), unsigned(!!(m&2)), unsigned(!!(m&4)), r, reasonName(r)); log_(text);
        prompt();
      }
      if (save && p == JC_SAVE_ARMED && r == JC_OK) {
        log_("[JOINT CAL] Saving; movement inhibited. Waiting for read-back verification\n");
        command("COMMIT", nullptr, now);
      }
      if (phase == JC_FAULT && reason == JC_LINK_ERROR) {
        pending = false; failure = "Nucleo Portenta UART receive fault";
      } else if (phase == JC_FAULT && reason == JC_ABORTED) {
        pending = false; failure = "Nucleo aborted joint calibration";
      } else if (phase == JC_FAULT && reason == JC_BAD_FEEDBACK) {
        pending = false; failure = "Nucleo feedback fault during calibration";
      } else if (phase == JC_IDLE) {
        // An unsolicited IDLE is a closed/lost session, not a live wizard.
        // Also handles older Nucleo firmware that reported an abort as IDLE/OK.
        pending = false;
        if (ack && !strcmp(verb_, "CANCEL") && reason == JC_OK) active = false;
        else failure = ack && !strcmp(verb_, "BEGIN") ?
          "Nucleo rejected joint calibration" : "Nucleo calibration session ended unexpectedly";
      }
      return;
    }
    unsigned long revision, build; float o[5]; unsigned src, captureAxis;
    if (sscanf(line, "H:%lu,%lu,%u,%f,%f,%f,%f,%lu%n", &session, &revision, &captureAxis,
        &o[0], &o[1], &o[2], &o[3], &build, &end) == 8 && !line[end] && captureAxis < 3) {
      if (active && session != session_) return;
      for (int i = 0; i < 4; ++i) if (!isfinite(o[i])) return;
      char text[176];
      snprintf(text, sizeof(text),
        "[JOINT CAL] Capture %s: raw L/R/elbow=%.3f,%.3f,%.3f reference=%.3f build=%08lX generation=%lu\n",
        captureAxis == 0 ? "PITCH" : captureAxis == 1 ? "ROLL" : "ELBOW",
        o[0],o[1],o[2],o[3],build,revision); log_(text); return;
    }
    if (sscanf(line, "O:%lu,%lu,%f,%f,%f,%f,%f,%u%n", &session, &revision,
          &o[0], &o[1], &o[2], &o[3], &o[4], &src, &end) != 8 || line[end]) return;
    for (float v : o) if (!isfinite(v) || fabsf(v) > 720) return;
    if (active && session != session_) return;
    generation = revision; source = src; statusReceived = true; memcpy(offsets, o, sizeof(o));
    char text[192];
    snprintf(text, sizeof(text),
      "[JOINT CAL] %s generation=%lu source=%s(%u) offsets L/R/elbow/q4/q5=%.3f,%.3f,%.3f,%.3f,%.3f\n",
      phase == JC_REVIEW ? "PROPOSED" : "ACTIVE", revision,
      src == JC_STORED ? "STORED" : src == JC_DEFAULTS ? "DEFAULTS" :
      src == JC_TEMPORARY ? "TEMPORARY" : "CORRUPT", src, o[0],o[1],o[2],o[3],o[4]);
    log_(text);
    if (phase == JC_REVIEW && haveBaseline_) {
      snprintf(text, sizeof(text), "[JOINT CAL] Changes L/R/elbow= %+.3f, %+.3f, %+.3f deg\n",
        o[0]-baseline_[0], o[1]-baseline_[1], o[2]-baseline_[2]); log_(text);
    } else if (active && !haveBaseline_) {
      memcpy(baseline_, o, sizeof(o)); haveBaseline_ = true;
    }
  }
  void select(unsigned nextAxis, int nextDirection, uint32_t now) {
    if (pending || (phase != JC_READY && phase != JC_STOPPED && phase != JC_REVIEW && phase != JC_LEAD_READY)) return;
    char args[24]; snprintf(args, sizeof(args), ",%u,%d", nextAxis, nextDirection);
    if (command("SELECT", args, now)) { axis = nextAxis; direction = nextDirection; printSelection(); }
  }
  void start(uint32_t now) {
    if (phase == JC_LEAD_READY) command("SWEEP", nullptr, now);
    else if (phase == JC_READY || phase == JC_STOPPED || phase == JC_REVIEW) command("MOVE", nullptr, now);
  }
  void stop(uint32_t now) {
    if (phase == JC_LEAD_IN || phase == JC_LEAD_READY || phase == JC_SWEEP || phase == JC_CAPTURING)
      command("STOP", nullptr, now, true);
  }
  void accept(uint32_t now) {
    if (phase == JC_READY || phase == JC_STOPPED) command("CAPTURE", nullptr, now);
    else if (phase == JC_REVIEW) command("APPLY", nullptr, now);
    else if (phase == JC_APPLIED) command("VERIFY", nullptr, now);
  }
  void cancel(uint32_t now) {
    if (!committing()) command("CANCEL", nullptr, now, true);
  }
  void save(uint32_t now) { if (phase == JC_VERIFIED) command("SAVE", nullptr, now); }
  void printSelection() {
    char text[112];
    snprintf(text, sizeof(text), "[JOINT CAL] Axis=%s approach=%c; left stick Y selects axis, X selects direction\n",
      axis == 0 ? "PITCH" : axis == 1 ? "ROLL" : "ELBOW", direction > 0 ? '+' : '-'); log_(text);
  }
  static const char *phaseName(unsigned p) {
    static const char *names[] = {"IDLE","READY","LEAD-IN","READY TO SWEEP","SWEEP","STOPPED",
      "CAPTURING","REVIEW","TEMPORARILY APPLIED","VERIFIED","SAVE ARMED","SAVING","SAVED","FAULT"};
    return p <= JC_FAULT ? names[p] : "UNKNOWN";
  }
  static const char *reasonName(unsigned r) {
    switch(r) {
    case JC_OK:return "OK"; case JC_BAD_COMMAND:return "invalid command";
    case JC_BAD_STATE:return "operation unavailable in this state";
    case JC_BAD_FEEDBACK:return "feedback invalid/outside limits";
    case JC_LEASE_EXPIRED:return "motion permission expired; cancel and re-enter";
    case JC_TRAVEL_LIMIT:return "travel/angle limit"; case JC_TIME_LIMIT:return "motion/loop timeout";
    case JC_STALLED:return "no measured progress"; case JC_CROSS_AXIS:return "other joint drifted";
    case JC_NOT_STILL:return "wait for stationary readings then retry";
    case JC_CAPTURE_MOVED:return "movement during capture; retry";
    case JC_BAD_VALUE:return "invalid calibration value"; case JC_STORAGE_ERROR:return "Flash verification failed";
    case JC_LINK_ERROR:return "Portenta UART/command receive fault; session aborted";
    case JC_ABORTED:return "Nucleo stopped or left calibration mode";
    default:return "calibration status";
    }
  }
private:
  Sender send_ = nullptr; Logger log_ = nullptr;
  uint32_t session_ = 0, request_ = 0, sent_ = 0, lastPing_ = 0, lastStatus_ = 0;
  unsigned attempts_ = 0; char command_[128]{}, verb_[16]{};
  bool haveBaseline_ = false; float baseline_[5]{};
  void prompt() {
    switch (phase) {
    case JC_READY:case JC_STOPPED:
      log_("[JOINT CAL] A: 10deg lead-in; X: capture physically verified reference after settling; L1: cancel\n"); break;
    case JC_LEAD_READY:log_("[JOINT CAL] A: continuous sweep toward reference; Y: stop\n"); break;
    case JC_LEAD_IN:case JC_SWEEP:log_("[JOINT CAL] Y: stop now; B: emergency\n"); break;
    case JC_REVIEW:log_("[JOINT CAL] Review proposed offsets; X: apply temporarily; L1: discard\n"); break;
    case JC_APPLIED:log_("[JOINT CAL] Check all physical references with level; X: confirm verification; L1: rollback\n"); break;
    case JC_VERIFIED:log_("[JOINT CAL] R1: SAVE after verifying arm supports itself; L1: rollback\n"); break;
    case JC_SAVED:log_("[JOINT CAL] SAVED and read back; L1: return to MANUAL_0. Recheck pickup-height fit before AUTO\n"); break;
    case JC_FAULT:log_("[JOINT CAL] Motion stopped; L1: rollback/exit; B: emergency\n"); break;
    default:break;
    }
  }
};
