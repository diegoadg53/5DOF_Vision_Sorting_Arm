#pragma once
#include "TrajectoryTestConfig.h"
#include "JointMotionTestConfig.h"
#include "JointDiagnosticProtocol.h"

class JointMotionTest {
public:
  using Sender = bool (*)(const char *);
  using Logger = void (*)(const char *);
  enum Buttons { Start=1, Pause=2, Cancel=4 };
  enum class State { Ready, Starting, InitialHome, Baseline, Picking, FinalHome,
                     Dwell, Ending, Finished, CancelHome };
  void bind(Sender s, Logger l) { send_=s; log_=l; }
  void enter(ArmTrajectory &arm, const RobotDHParams &params, const AutoConfig &base,
             unsigned repeats=JointMotionTestSettings::cycles) {
    Sender s=send_; Logger l=log_; *this=JointMotionTest{}; bind(s,l);
    arm_=&arm; params_=&params;
    repeats_=repeats;
    config_=TrajectoryTest::configuration(base,JointMotionTestSettings::pickupXmm,params,
                                         JointMotionTestSettings::binId);
    for (const auto &route : Classification::partRoutes)
      if (route.binId==JointMotionTestSettings::binId &&
          Classification::destinationForPart(config_,route.partId)) { partId_=route.partId; break; }
    arm_->cancel();
    char text[176]; snprintf(text,sizeof(text),
      "[JOINT TEST] %u cycles, X=%.1fmm, bin=%u; baseline/HOME dwells=%lu/%lums\n",
      repeats_,JointMotionTestSettings::pickupXmm,
      unsigned(JointMotionTestSettings::binId),
      (unsigned long)JointMotionTestSettings::baselineDwellMs,
      (unsigned long)JointMotionTestSettings::homeDwellMs); log_(text);
    log_("[JOINT TEST] Start Python on Nucleo USB first. A: start/resume; Y: pause; L1: cancel/HOME; B: emergency\n");
  }
  void receive(const char *line, uint32_t now) {
    unsigned long run; unsigned active,interval,size; int end=0;
    if (sscanf(line,"D:%lu,%u,%u,%u%n",&run,&active,&interval,&size,&end)!=4 ||
        line[end] || !run_ || run!=run_ || active>1 || interval!=JD_INTERVAL_MS ||
        size!=sizeof(JointDiagnosticPacket)) return;
    ackActive_=active; lastAck_=now; haveAck_=true;
  }
  void emergency(uint32_t now) {
    if (run_) { mark(JD_FAULT,now); sendStop(now); }
    running_=false; arm_=nullptr;
  }
  const char *fault() const { return fault_; }
  bool exitRequested() const { return exit_; }
  State state() const { return state_; }
  unsigned cycle() const { return cycle_; }
  uint32_t runId() const { return run_; }
  bool active() const { return running_; }
  const AutoConfig &configuration() const { return config_; }
  void tick(const ArmFeedback &f, uint32_t now, unsigned buttons, uint32_t nonce) {
    if (!arm_ || fault_) return;
    if (running_) {
      if (uint32_t(now-lastPing_)>=250) {
        char b[64]; snprintf(b,sizeof(b),"D:PING,%lu\n",(unsigned long)run_);
        lastPing_=now; write(b);
      }
      if (uint32_t(now-lastAck_)>1500) { fail("Joint-test recorder acknowledgement lost"); return; }
    }
    if ((buttons&Cancel) && state_!=State::CancelHome) {
      if (state_==State::Ending || state_==State::Finished) {
        sendStop(now); running_=false; exit_=true; return;
      }
      exit_=false; pausedHold_=false;
      mark(JD_CANCEL_HOME,now);
      if (arm_->busy()) arm_->cancelPickup(f,now);
      else if (!arm_->prepare(config_,f,now,arm_->clawMayHoldPart())) { fail(arm_->fault()); return; }
      state_=State::CancelHome;
    } else if (buttons&Pause) {
      if (arm_->busy() && !arm_->paused()) arm_->requestPause(now);
      else if ((state_==State::Baseline || state_==State::Dwell) && !pausedHold_) {
        pausedHold_=true; pauseAt_=now; mark(JD_PAUSED,now);
      }
    } else if (buttons&Start) {
      if (arm_->paused()) { arm_->resume(now); markArm(now); }
      else if (pausedHold_) {
        dwellSince_+=uint32_t(now-pauseAt_); pausedHold_=false;
        mark(state_==State::Baseline ? JD_BASELINE_HOME : JD_HOME_DWELL,now);
      } else if (state_==State::Ready || state_==State::Finished) {
        if (!partId_ || !repeats_ || repeats_>10000 || !validArrivalConfiguration(config_) ||
            !isfinite(config_.pickupZ)) { fail("Invalid joint-test route/configuration"); return; }
        run_=nonce ? nonce : 1; cycle_=0; haveAck_=ackActive_=false;
        lastAck_=lastPing_=lastStart_=now; running_=true; exit_=false;
        state_=State::Starting;
        sendStart(now);
      }
    }
    if (fault_) return;
    if (state_==State::Starting) {
      if (haveAck_ && ackActive_) {
        mark(JD_INITIAL_HOME,now);
        if (!arm_->prepare(config_,f,now)) { fail(arm_->fault()); return; }
        state_=State::InitialHome;
      } else if (uint32_t(now-lastStart_)>=300) sendStart(now);
    } else if (state_==State::Ending) {
      if (haveAck_ && !ackActive_) {
        running_=false; state_=State::Finished;
        log_("[JOINT TEST] Complete at HOME; recorder stopped. A: new run; L1: exit\n");
      } else if (uint32_t(now-lastStop_)>=300) sendStop(now);
    }
    if (state_==State::Baseline || state_==State::Dwell) {
      if (!f.valid || uint32_t(now-f.timestampMs)>config_.feedbackTimeoutMs) {
        fail("Joint-test HOME dwell feedback stale"); return;
      }
      for (unsigned i=0;i<5;++i)
        if (!isfinite(f.q[i]) || fabsf(f.q[i])>JointMotionTestSettings::maximumHomeJointDriftDeg) {
          fail("Joint-test measured HOME drift exceeded limit"); return;
        }
      const uint32_t duration = state_==State::Baseline ? JointMotionTestSettings::baselineDwellMs :
                                                        JointMotionTestSettings::homeDwellMs;
      if (!pausedHold_ && uint32_t(now-dwellSince_)>=duration) {
        if (cycle_>=repeats_) {
          mark(JD_COMPLETE,now); haveAck_=false; sendStop(now); state_=State::Ending;
        } else startCycle(f,now);
      }
    }
    arm_->tick(f,now);
    uint32_t id=0; PickOutcome outcome=arm_->takeOutcome(id);
    if (outcome==PickOutcome::Fault) { fail(arm_->fault()); return; }
    if (outcome==PickOutcome::Missed || outcome==PickOutcome::Rejected) {
      if (outcome==PickOutcome::Missed && state_==State::CancelHome) {
        sendStop(now); running_=false; exit_=true; state_=State::Ready; return;
      }
      fail(outcome==PickOutcome::Rejected ? "Joint-test plan rejected" : "Joint-test pickup deadline missed"); return;
    }
    if (outcome==PickOutcome::HomeReady) {
      if (state_==State::CancelHome) {
        sendStop(now); running_=false; exit_=true; state_=State::Ready;
      } else {
        state_=state_==State::InitialHome ? State::Baseline : State::Dwell;
        dwellSince_=now; mark(state_==State::Baseline ? JD_BASELINE_HOME : JD_HOME_DWELL,now);
        char b[112]; snprintf(b,sizeof(b),"[JOINT TEST] HOME measurement: cycle=%u/%u, run=%lu\n",
            cycle_,repeats_,(unsigned long)run_); log_(b);
      }
    } else if (outcome==PickOutcome::Sorted) {
      mark(JD_FINAL_HOME,now);
      if (!arm_->prepare(config_,f,now)) { fail(arm_->fault()); return; }
      state_=State::FinalHome;
    }
    if (arm_->busy() && (state_==State::Picking || arm_->paused())) markArm(now);
    if (running_ && haveAck_ && !ackActive_ && state_!=State::Ending && state_!=State::Starting)
      fail("Nucleo recorder stopped during joint test");
  }
private:
  Sender send_=nullptr; Logger log_=nullptr;
  ArmTrajectory *arm_=nullptr; const RobotDHParams *params_=nullptr;
  AutoConfig config_;
  State state_=State::Ready;
  const char *fault_=nullptr;
  uint32_t run_=0,lastAck_=0,lastPing_=0,lastStart_=0,lastStop_=0,dwellSince_=0,pauseAt_=0;
  unsigned cycle_=0,lastStage_=~0U;
  unsigned repeats_=JointMotionTestSettings::cycles;
  uint8_t partId_=0;
  bool running_=false,haveAck_=false,ackActive_=false,exit_=false,pausedHold_=false;
  void fail(const char *reason) { fault_=reason && *reason ? reason : "Joint-test motion failure"; }
  void write(const char *text) { if (!send_(text)) fail("Joint-test UART write failed"); }
  void sendStart(uint32_t now) {
    char b[64]; snprintf(b,sizeof(b),"D:START,%lu\n",(unsigned long)run_); lastStart_=now; write(b);
  }
  void sendStop(uint32_t now) {
    if (!run_) return;
    char b[64]; snprintf(b,sizeof(b),"D:STOP,%lu\n",(unsigned long)run_); lastStop_=now; write(b);
  }
  void mark(unsigned stage,uint32_t now) {
    (void)now;
    if (!running_ || lastStage_==stage) return;
    lastStage_=stage; char b[96]; snprintf(b,sizeof(b),"D:MARK,%lu,%u,%u\n",
      (unsigned long)run_,cycle_,stage); write(b);
  }
  void markArm(uint32_t now) {
    const char *p=arm_->phaseName(); unsigned stage=JD_PLAN;
    if (arm_->paused()) stage=JD_PAUSED;
    else if (!strcmp(p,"APPROACH")) stage=JD_APPROACH;
    else if (!strcmp(p,"DESCEND")) stage=JD_DESCEND;
    else if (!strcmp(p,"WAIT FOR PART")) stage=JD_WAIT;
    else if (!strcmp(p,"CLOSE")) stage=JD_CLOSE;
    else if (!strcmp(p,"CLASSIFICATION WAYPOINT")) stage=JD_WAYPOINT;
    else if (!strcmp(p,"CLASSIFICATION POSE")) stage=JD_DROP;
    else if (!strcmp(p,"OPEN")) stage=JD_OPEN;
    else if (!strcmp(p,"HOME")) stage=state_==State::FinalHome ? JD_FINAL_HOME : JD_INITIAL_HOME;
    mark(stage,now);
  }
  void startCycle(const ArmFeedback &f,uint32_t now) {
    ++cycle_; lastStage_=~0U; mark(JD_PLAN,now);
    Part p; p.trackId=cycle_; p.inUse=true; p.hits=3; p.classId=partId_;
    p.x=JointMotionTestSettings::pickupXmm; p.y=TrajectoryTest::initialYmm;
    p.boxWidth=20; p.boxHeight=40; // Repeat the validated tall-part orientation.
    p.stateMs=p.observedMs=now;
    const Destination *destination=Classification::destinationForPart(config_,partId_);
    if (!destination || !arm_->start(p,TrajectoryTest::velocityMmS,destination->pose,config_,f,now)) {
      fail("Joint-test cycle could not start"); return;
    }
    state_=State::Picking; char b[96]; snprintf(b,sizeof(b),"[JOINT TEST] Cycle %u/%u\n",
        cycle_,repeats_); log_(b);
  }
};
