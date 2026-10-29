# Portenta V3 automatic mode

Runtime controller tuning and persistent camera/belt calibrations are described
in [RUNTIME_TUNING.md](RUNTIME_TUNING.md). CALIBRATION MENU now includes TUNING &
STORAGE; R3 from AUTO/TEST requests that menu after the cycle returns HOME.
AUTO PICKUP TUNING runs ID-ignorant/model-131/arm AUTO with separate WIDE/TALL
pickup settings, editable only at IDLE. The nine timing/height/X-correction
fields moved there; the TALL profile also has an ID 1 extra X correction. Shared
reference X and synthetic test controls stay in
TUNING & STORAGE. Both profiles persist with the camera/belt calibrations.
The ID 1 TALL extra is also available in TUNING & STORAGE via R3 from AUTO, so it
can be adjusted between ID-classification trials without entering an ID-ignorant run.

## Coordinates and joint order

All Cartesian targets and `robot_fkine()` results now describe the TCP. The tool
translation is `(0, 0, -49)` mm in the final DH frame's local axes. It rotates
with the tool; it is not a fixed translation along base Z.

The previous zero-roll, zero-pitch DH-origin targets convert to pickup Z = -34
mm, approach Z = 51 mm, and classification pose `{0,200,251,0,0,0}`. These are
converted starting targets, not a newly measured belt surface.
The sorting layout is configured in `ClassificationConfig.h`; the current
reference TCP is {-76,300,170,0,14.545,90}, with roll offsets -40, -20 and 0 degrees.
See [CLASSIFICATION_WORKSPACE.md](CLASSIFICATION_WORKSPACE.md) for the constrained workspace.

Joint arrays use `pitch,roll,j3,j4,j5`, matching Nucleo `A:` telemetry. The command
adapter sends `P:roll,pitch,j3,j4,j5`. Joint limits follow the internal array order.
Cartesian poses use `x,y,z,roll,pitch,yaw`, with mm and degrees respectively, and
the ZYX Euler convention. AUTO and the trajectory test enforce the same full
base-X tilt. Approach uses yaw 0; pickup chooses yaw 0 or 90 from the last
accepted camera bounding box. MANUAL_1 uses pose-dependent
orientation relaxation while requiring TCP position within tolerance. Configured
ring classification enforces XYZ and full TCP orientation with shoulder pitch=0
and shoulder roll fixed at the bin's radial angle. q3/q4/q5 solve the final pose.
The shoulder roll joint still moves to the selected bin during classification.

## IK convergence and orientation relaxation

The geometric Jacobian includes the local -49 mm tool offset. For MANUAL_1 and
classification without locked shoulder pitch, the solver computes a basis of `null(J_position)`, then the
attainable angular-velocity subspace `range(J_rotation * null(J_position))`.
The rotational error **and** rotational Jacobian are projected onto that
subspace at every iterate. This identifies the unavailable rotation from the
current pose rather than assuming it is always Euler roll, pitch or yaw.
TCP position error is never projected away. This follows the Jacobian/null-space
task formulation described in [Siciliano's kinematic-control tutorial](https://doi.org/10.1007/BF00126069).

The angular part of a true left-null vector is a valid alternative description
at a regular pose. If `n=[n_position; n_rotation]` satisfies `J^T n=0`, then
`n_rotation^T J_rotation dq=0` for every `dq` with `J_position dq=0`.
With position rank 3 and full Jacobian rank 5, normalizing `n_rotation` gives
the one unavailable rotation direction while holding TCP position. Removing
that direction agrees with the projector used here. It is not generally a
fixed Euler axis. At lower rank, a single direction may no longer suffice.
Projecting the complete six-component error instead can alter position demands;
this implementation keeps all three position residuals in the convergence test.

This is a local numerical orientation relaxation; it does not establish the
globally nearest orientation or make every position reachable within joint
limits. At singular configurations, the attainable rotation subspace can also
lose rank. Pickup retains the separately requested hard pitch/yaw constraints.

The command is rounded to 0.01 degrees and FK is checked again before success.
Successful degree output retains that precision after conversion from radians.
Motion admission compares float radians against the stored float-radian limits,
matching IK's validation. This prevents a valid 84.00-degree wrist command from
being returned as 84.000008 and falsely rejected at its 84-degree upper limit.
Joint limits are not widened.
Previously, a floating-point solution inside 0.5 mm could fail immediately when
its rounded command exceeded that tolerance slightly. The solver now continues
refining until the rounded command passes, or the iteration budget is exhausted.
The 0.5 mm and 0.035 rad tolerances and physical joint limits remain unchanged.

MANUAL_1 solves each increment from its last accepted joint command, avoiding
seed variation from motor lag/noise. A rejected increment changes neither the
Cartesian target nor the command seed, and sends no new position command. It
does not itself trigger ESTOP, so the operator can reverse away from the rejected
request. Connection loss, the emergency button and unavailable motion feedback
still stop the process. The displayed IK pose is the requested pose; REAL TCP is
the achieved pose, so their orientations may differ under this relaxation.

`[IK MANUAL]` reports rejected increments or orientation relaxation, with
`controlledRotError` and `fullRotError` separately. The former is the residual
of the controlled rotation task; the latter includes the unavailable component.
`[PLAN] Rejected: ...` distinguishes a pickup deadline rejection from IK failure
and a classification waypoint outside limits. `[IK APPROACH]`, `[IK PICKUP]` or
`[IK CLASSIFICATION]` includes the task, failure reason, iteration count, TCP
request, seed and remaining errors; `[PLAN] candidate` prints the last candidate
joints. A deadline rejection can occur with an IK solver still running.

The main sketch's `max_iterations` applies to MANUAL_1; AUTO/test planning has
its own 150-iteration budget per target. Increasing either limit cannot resolve
an unreachable task, and did not affect the previous rounding early exit.

## Modules and operation

- `BeltTracker` predicts all tracks every loop and incorporates new observations
  one-to-one. A camera class label and a unique Portenta track ID are separate.
- `ArmTrajectory` owns an assigned part snapshot, performs IK incrementally, and
  supervises one `P:` command per point. No Cartesian interpolation is added.
- `PickGeometryDefaults.h` holds build geometry defaults and the fixed slope/X
  range; `PickMotionConfig.h` applies runtime geometry and sets wrist allowance
  and HOME/waypoint TCP band. `AutoPickSelection.h` evaluates real
  tracked parts against routing, workspace and the preparation-time budget.
- The main sketch owns menus, communication transactions, normal Stop, and the
  emergency latch. Tracking and arm supervision are serviced separately.

Joint zero calibration is available from MANUAL_0 with L3+R3 held for one
second. See [JOINT_CALIBRATION.md](JOINT_CALIBRATION.md) for the guided sweeps,
runtime offset application, persistence and recovery. Recheck the physical
pickup-height fit after teaching different joint zeros.

Choose the camera model, ID-ignorant/classification option, and MOVE ARM/ONLY
VISION in AUTO_MENU. X selects the options and enters AUTO_MODE. AUTO_MODE first
acknowledges stream shutdown to synchronize with the ESP32.

- A starts. MOVE ARM first homes and opens the gripper under supervision.
- Y finishes the assigned pick/sort (or missed-pick recovery), returns HOME,
  then stops the stream. It never assigns a new part after Stop is requested.
- L1 does the same and returns to AUTO_MENU. Stop/exit beat A if pressed in the
  same controller batch. Neither button freezes real part timing.
- The emergency button, controller loss, camera offline event, failed ESP32
  transaction, stale active-arm telemetry, or arm fault latches an emergency.
- Hold L1+R2 or R1+L2 for three seconds with a connected controller and the
  emergency button released to reset. Reset returns to MODE_SELECTOR; it never
  resumes a job.

ONLY VISION maintains the tracker without issuing home, position, gripper, or
`S:AUTO` commands. ID-ignorant operation uses the default, farthest bin.
ONLY VISION still requires A after entry to start streaming. The console
reports the START request/acknowledgement, then sampled `[VISION]` received
IDs, pixel/world coordinates and candidate counts every 500 ms. It explicitly
reports missing/stale detection packets. See the ESP32 project's
`CLASSIFICATION_DIAGNOSTIC.md` for corresponding camera polling/send counters;
absence of `[AUTO] Assigned` in ONLY VISION is expected.
ID-aware operation maps camera part IDs to enabled bins through
`ClassificationConfig.h`. Before reserving a part or planning its pickup,
ID CLASSIFICATION waits for a stable label. Edit
`CameraTracking::ClassificationConfidence` in `CameraTrackingConfig.h`:

```cpp
constexpr uint32_t minimumStableMs = 600;
constexpr uint8_t minimumSamples = 3;
constexpr uint32_t sampleIntervalMs = 80, maximumGapMs = 750;
```

The SAME ID must be observed at changed pixel centres at least three times,
spanning at least 600 ms of accepted observations. Another ID immediately resets
the confirmation window. A gap over 750 ms before a new changed-position
observation restarts it. Consecutive held centres do not increment the count or
extend the span; box-size jitter is ignored. New coordinates received too soon
are discarded rather than accepted later as held copies. An ID change still
resets immediately, including when its centre is unchanged. Clock time passing
without new evidence never confirms a label.

The 600 ms starting value exceeds the roughly 550 ms initial-ID correction
window seen in the supplied ESP32 logs. It is a tunable observation filter,
not proof of a correct class or a camera inference-frame counter. Location/label
jitter can still look like new evidence, and a persistently wrong camera class
can still pass. No camera-confidence field or frame timestamp was added to the
wire protocol.

An early unmapped label remains pending so it can be corrected. A confirmed
unmapped/explicitly ignored ID or disabled target
bin is retired without a pickup and increments `ignoredParts`; it is distinct
from an IK/trajectory rejection. The preparation-time budget is enforced while
waiting: an unconfirmed part that reaches the admission deadline is retired as
missed, without starting a pickup. Waiting never pauses the real belt clock or
reduces the configured arrival-time pickup budget. A previously confirmed label
can remain usable after the part leaves view, within the existing track-age
and deadline checks; a newer conflicting label revokes confirmation.

`[AUTO] Waiting for classification` reports a pending track/ID, accepted sample
count and observed span at most once per 500 ms across waiting tracks.
`Classification locked` reports the confirmed ID/bin and evidence when assigned.
After reservation the ID, bin, unshifted camera X and Y/time snapshot remain fixed.
The assigned track's last accepted box selects its WIDE/TALL profile until descent.
That profile determines X correction, heights, approach budget and gripper timing,
as well as yaw. Both sets of targets are preplanned; changing safety X/Z during
approach requires reconfirming the new safety pose. An unavailable profile or
late safety arrival recovers HOME as a missed pickup. Profile and orientation
lock together at descent. ID IGNORANT bypasses the classification gate, and the synthetic
trajectory test still uses its configured mapped ID.

The three bin targets are initialized from the
reference pose and configured roll offsets. The two generated poses are model
targets for physical testing, not independently measured bin locations.

The scheduled pickup sequence reserves/plans at HOME, waits for approach time,
reaches and settles at safety height, waits for descent time, then overlaps descent
and closing as configured. It does not wait for gripping-height arrival to issue
the timed close command. Both WIDE and TALL pickups use timed completion:
transport starts after the actual close command's full closing duration plus
100 ms, without requiring gripping-height position/settling confirmation.
Every missed/cancelled pickup suppresses all pending pickup timers and returns
through the approach point (if already moving) to supervised HOME. If closing
has started, it finishes plus the 100 ms delay before lifting, and no open
command is sent in recovery.
AUTO stops for operator intervention in that case; A acknowledges and opens
before restarting. An open-claw miss resumes AUTO selection after HOME. The test
waits for A after recovery; L1 cancellation returns to the calibration menu.
Tracking continues throughout recovery. The existing
two-command classification structure is retained. At the waypoint, shoulder
pitch/roll go to zero, elbow and q4 preserve their latest measured pickup
positions, and q5 retains its pickup command. q4 and q5 change to their final drop
angles only after the waypoint settles. Configured ring sorting uses full-pose IK
with mask 0x03 holding shoulder pitch=0 and roll at the bin angle; only q3/q4/q5
are solved. No shoulder command is overwritten after IK. Fixed columns
are removed from the solver's Jacobian and locked seeds must be inside limits.
Position tolerance and quantized command checks remain active. The changed
waypoint and generated bin motions need physical path validation.

### Real AUTO admission and shared motion

AUTO uses the track's measured X, predicted Y, timestamp and camera class ID,
plus the signed calibrated belt velocity. It does not randomize these values.
`serviceAutomaticMode()` services tracking separately from arm supervision.
At startup it waits for stream acknowledgement and supervised HOME/open before
reserving a part. It then selects the eligible part with the earliest arrival.
After sorting/opening or open-claw missed-pick recovery, it waits for supervised
HOME before selecting again. A recovery with the claw closed stops AUTO. A plan rejected before motion retires the track and supervises HOME before
selecting again.

The supported pickup range is the editable `PickMotion` range, currently
200 <= X <= 450 mm. Outside tracks are retired
without motion and counted in `outsidePickupParts` and `ignoredParts`; the fit
is not extrapolated. Unknown/unmapped IDs are ignored only in ID-aware mode.
ID-ignorant mode sorts every eligible ID to the default bin. ONLY VISION keeps
tracking without calling motion selection or issuing HOME/position/gripper
commands; its startup does not require joint limits or arm feedback.

Timing is shared between AUTO and TRAJECTORY TEST. The current build defaults
in `AutoConfig` (`ArmTrajectory.h`) are below; applied/saved controller tuning
overrides them independently for WIDE/TALL. Use AUTO PICKUP TUNING to edit without reflashing:

```cpp
descentLeadMs = 800;
closingLeadMs = 400;
closingDurationMs = 520;
pickUpMarginMs = 1600;
// Derived: pickUpBudgetMs() = margin + max(descentLead, closingLead) = 2400 ms.
```

All times below are relative to the predicted arrival at fixed robot Y=0:

| Event | Scheduled time |
| --- | ---: |
| Start moving to safety height | -2400 ms |
| Start descent and final pickup orientation | -800 ms |
| Start closing (independent of descent completion) | -400 ms |
| WIDE/TALL: transport after closing duration + 100 ms | +220 ms |

Selection requires **remaining arrival time >= pickup budget**, plus the existing
stable-class, routing and workspace checks; choose the earliest eligible arrival.
Reservation/planning happens early, but approach waits until its scheduled time.
Both profiles' safety/pickup targets and classification solutions are preplanned before any
approach movement. Planning that runs beyond the scheduled approach is a missed
pickup, not permission to start late. There is no old 3000/5000 ms admission gate.
With the same default leads, changing the margin changes only approach start;
changing descentLead changes both descent time and the derived approach budget.
The margin must cover actual approach travel/settling. Both pickup profiles use
the timed completion policy below, so descent lead can account for the
part/gripper contact geometry rather than a gripping-height settling deadline.
These are physical tuning parameters.
Timing validation requires descentLead >= closingLead and closingDuration >=
closingLead. It rejects zero/invalid required timers and unbounded lead values.

Safety-height confirmation (including settling) must occur with at least
`descentLeadMs` remaining. Otherwise `[PICK MISSED] reason=SAFETY_HEIGHT_LATE`
records the remaining time and recovers. Equality is allowed. A separate editable
`scheduleToleranceMs=25` allows loop jitter in issuing approach/descent commands;
it never relaxes the safety-confirmation condition. Existing `maximumLateMs`
and `maximumCaptureErrorYmm` bound late close commands separately.
Other reasons include PLAN_LATE, APPROACH_COMMAND_LATE, DESCENT_COMMAND_LATE,
SAFETY_HEIGHT_LOST, CLOSE_COMMAND_LATE, GRIPPING_HEIGHT_LATE and PICKUP_POSE_LOST.
Feedback loss, command failures, or a timed-out movement/recovery still latch ESTOP.

### WIDE and TALL pickup completion by time

Once the last accepted box is locked at descent, **WIDE (height<=width/yaw0)**
or **TALL (height>width/yaw90)**
uses time alone to complete pickup. It does not require gripping-height joint/TCP
tolerance, settling, minimum movement time or confirmation by Y=0, and does not
cancel for PICKUP_POSE_LOST during that pickup. Contact can disturb the arm while
closing. The controller continues to report the pose errors for observation.

The classification waypoint starts when:

```text
elapsed since actual close command >= closingDurationMs + pickupPostCloseMs
pickupPostCloseMs = 100 ms
```

For closingLead=360 ms and closingDuration=600 ms, this is nominally 340 ms
after predicted Y=0. Closing lead is an offset before arrival, not an additional
wait after issuing close. The extra 100 ms lives in `PickMotionConfig.h`; the
existing per-profile closing duration is still editable in AUTO PICKUP TUNING.
The extra delay is shared by WIDE and TALL; closing duration remains independent
per profile. The code setting is now named `PickMotion::pickupPostCloseMs`
(previously `widePickupPostCloseMs`). This does not change the saved settings schema.

Safety-height settling, scheduling/late-close checks, fresh feedback, command
joint limits, stage timeouts, emergency stops and subsequent classification/HOME
position checks still apply. Timed completion does not prove the part was grasped
or that the gripping position was reached. The waypoint retains the latest
measured q3/q4 and the pickup q5 command, as before.

Y in TRAJECTORY TEST after closing starts waits through closing plus 100 ms and
then pauses; A resumes toward the waypoint. Before closing, Y still waits for
the commanded movement to settle and suppresses a new close command. A controlled
cancel after closing also finishes closing plus the 100 ms before recovery.
Both profiles retain position/settling checks at safety height and all movements
after pickup. The optional legacy position-verified controller configuration
still uses gripping-height settling when timed completion is disabled.

### Reading pickup timing and missed-pick diagnostics

`[PICK TIMING]` emits one event at safety confirmation, the descent command,
the close command. Each profile reports `[PICK POLICY]` at descent and either
`[PICK TIMING] WIDE_TIMED_COMPLETE` or `TALL_TIMED_COMPLETE` before transport,
including actual close elapsed time, required closing-plus-post-close time and
observed TCP error; it never labels timed completion as confirmed position.
`remainingArrival` is signed
milliseconds until predicted Y=0. `targetElapsed` is time since the current
position target was commanded; CLOSE still refers to the gripping-height target.
`stable=50/150ms` means only 50 ms of the required 150 ms of stable feedback has
been confirmed. Settling and minimum movement duration are concurrent checks,
not two delays added after the move. `deadlineMargin` is remaining arrival time
minus that event's threshold: positive means headroom, negative means lateness.

Every missed-pick diagnostic captures the failed target, errors, configuration
and feedback BEFORE recovery can replace them. These lines are emitted once,
independently of the normal one-second `[ARM]` print interval:

- `[PICK MISSED]`: original phase/target, track, reason, remaining arrival and claw state.
- `[PICK DEADLINE]`: the requirement relevant to this failure, its remaining-time
  threshold, signed margin and permitted command lateness (if applicable).
- `[PICK CHECK]`: feedback freshness, joint/TCP checks, actual TCP error and
  active tolerance, stable/required duration, target elapsed/minimum duration.
- `[PICK JOINTS]`: original target, measured-minus-target error and tolerance
  in pitch/roll/j3/q4/q5 order. `outsideMask` bits 0..4 identify joints outside
  tolerance (0x08 means q4, 0x00 means no joint violation).
- `[PICK TUNING]`: this job's actual approach margin, descent/closing leads,
  closing duration, derived budget and elapsed time since close was commanded.

For SAFETY_HEIGHT_LATE, `thresholdRemaining=600.0ms` means safety confirmation
was required before descent time. For GRIPPING_HEIGHT_LATE, the requirement is
`GRIPPING_CONFIRMED_BY_Y0`, with `thresholdRemaining=0.0ms`; there is no additional
600 ms requirement after descending. That gripping deadline applies only to
legacy position-verified pickups, not the timed WIDE/TALL policy. For example, `jointsOK=1 TCPok=1` with
`stable=50/150ms` at that deadline identifies unfinished settling. A failing
joint/TCP check identifies positioning error instead.

`tickGap` and `maxTickGap` measure gaps between actual arm-controller calls;
`feedbackInterval` measures timestamp spacing between the most recently observed
feedback updates. They are not console print periods. The maximum resets on each
HOME preparation or assigned pickup. A low current tick gap does not exclude a
prior stall; inspect the maximum as well. These diagnostics do not change timing,
arrival tolerances, recovery commands or the current operator tuning.

### Tracking rectangle and last-box orientation

`CameraTracking::PartRegion::rectangle` in `CameraTrackingConfig.h` defaults to
`{64,48,576,432}`, the central 80% of the 640x480 image. The **whole bounding box**
must fit, before homography and X correction. Rejected boxes cannot create/update
tracks, labels or orientations. Prediction continues at the calibrated speed.
This is separate from the narrow AprilTag velocity ROI.

The last accepted box of the assigned track selects pickup yaw at descent:
height > width selects 90 degrees; height <= width selects 0 degrees. Missing or
clipped observations leave the previous accepted dimensions intact. Reserved
tracks expose box metadata without changing the reserved unshifted X/time
snapshot or classification/bin. Profile-dependent correction can change the
commanded X before descent. The final choice is logged as `[PICK] Orientation locked`.
This is a pixel-axis heuristic, without averaging/hysteresis or a world-angle
estimate; nearly square boxes can select either branch as measurements fluctuate.
At descent, the latest accepted branch becomes final for that pick.

For tilt alpha=atan(heightSlope), the ZYX orientations are:

| Target | Roll | Pitch | Yaw |
| --- | ---: | ---: | ---: |
| Safety height | 0 | -alpha | 0 |
| Wide/square pickup | 0 | -alpha | 0 |
| Tall pickup | -alpha | 0 | 90 |

This retains tilt along the same base-X direction. Classification keeps the
chosen pickup q5 and measured pickup q4 through the
waypoint. Existing geometry, X range, bin routes and tolerances are retained.

`[AUTO] Assigned` logs the chosen track/ID/bin, X/Y, corrected pickup Z, calibrated
velocity and remaining close delay. `[AUTO] Stage` and `[ARM]` progress expose
the same motion supervision as the test. Sorted/missed/rejected/ignored/outside
totals accumulate since boot or the last operator reset; visionUpdates counts
processed snapshots in the current stream session. To clear all six displayed
counters, press **Y** to finish the assigned part and return HOME/stop, then
press **L3 alone** while STOPPED. The reset also works in ONLY VISION and AUTO
PICKUP TUNING. It does not change tuning, calibrations, routes or storage, and
does not start motion or camera streaming. A reset request while running is
rejected immediately rather than queued; button chords do not clear counters.
A return-HOME failure or any existing controller/camera/feedback fault
latches ESTOP; a resumed link never resumes the job automatically.

## Classification bins and part IDs

`ClassificationConfig.h` is the shared editable configuration for AUTO and
TRAJECTORY TEST. Its current `referencePose` is TCP
{-76,300,170,0,14.545,90}. At shoulder pitch=0, the model's roll axis is vertical
through X=-76, Y=0. Bins 1, 2 and 3 use offsets -40, -20 and 0 degrees from
that reference, keeping the same radius/height. Bin 3 is the far end of the sweep.
Use the current header as the source of truth for user-tuned coordinates.

Coordinates are TCP mm. Change the
reference XYZ to change radius/height, and `bins[].rollOffsetDegrees` to change
angular spacing. Positions are rotated about the roll axis derived from the DH
model at shoulder pitch=0, rather than about base (0,0). Each bin has an editable
`enabled` flag. The reference orientation rotates with the bin, so the current
RPY targets are {0,14.545,50}, {0,14.545,70}, {0,14.545,90}. These angles are enforced
together with XYZ using fixed shoulder pitch/roll and free q3/q4/q5. Changing
reference Y/Z independently can make the full pose unreachable; see the arc and
ranges in [CLASSIFICATION_WORKSPACE.md](CLASSIFICATION_WORKSPACE.md).

Edit/append `partRoutes` to map IDs independently from the three positions:

```cpp
constexpr PartRoute partRoutes[] = {
    {2, 1}, {5, 2}, {8, 3}, // Current camera IDs.
    // {4, 1},             // Another ID can share bin 1.
    // {9, 0},             // Explicitly ignore ID 9.
};
```

IDs absent from this table are ignored in ID-aware AUTO. Duplicate part IDs or
out-of-range bin IDs reject ID-aware admission. TRAJECTORY TEST randomly chooses
among mapped IDs with enabled bins. Adding more IDs sharing a bin changes their
sampling frequency; the three current routes give equal bin chances.
The chosen ID/bin and X remain fixed throughout the entire cycle, including
HOME preparation and pauses. No eligible routes prevents a new test but does
not prevent L1 HOME/cancel.

## Timing and calibration

### Camera-independent trajectory test

In MODE_SELECTOR, L3+R3 opens CALIBRATION_MENU even with the camera offline.
Select TRAJECTORY TEST with the left joystick and X. Only the camera calibration
and velocity estimation menu entries require the camera.

If the camera initializes before the ESP32 joins Wi-Fi, the ESP32 must still
enter `WAITING_CAMERA_ONLINE_ACK`. Previously that startup branch entered
`NORMAL_COMMUNICATION` directly and never sent the initial `EVT_CAM_ONLINE`;
Portenta consequently retained its initial offline/unknown flag. The ESP32
startup branch now uses the existing acknowledged event exchange before normal
communication, just like camera recovery. This fix requires uploading the
updated `firmware/ESP32_V3` sketch as well as the Portenta arrival changes.

- A independently chooses integer X in the configured `PickMotion` range
  (currently 200..450 mm), an enabled mapped ID/bin, and tall/wide box orientation.
  It prepares HOME/open, then releases the imaginary part at Y=400 mm, -100 mm/s.
  `PickGeometryDefaults.h` supplies initial height/tilt defaults, overridden by
  applied/saved runtime tuning:
  `pickupZ = -40.104 + 0.135678 * (X - 300)` mm, with zero additional adjustment.
  Negative `pickupHeightAdjustmentMm` lowers TCP, positive raises it. The tilt
  alpha is atan(0.135678), about 7.727 degrees. The common approach is
  {roll=0,pitch=-alpha,yaw=0}; pickup yaw 0 uses that orientation, while yaw 90
  uses {roll=-alpha,pitch=0,yaw=90}. Approach/pickup use FullPose IK.
  These are absolute TCP coordinates; no extra tool or part-height offset is added.
  Approach Z retains the configured 51 mm. Both orientations and their bin
  solutions are planned before motion and the new combined motions need physical
  validation. Existing pickup-fit calibration remains empirical.
  The test classification target is the selected bin's full pose in
  `ClassificationConfig.h`. Shoulder pitch=0 and the selected shoulder-roll angle
  are locked inside IK while q3/q4/q5 solve XYZ and orientation. q4/q5 retain their
  pickup values at the waypoint and change at final classification. Full pose
  failure rejects planning before approach. `[CLASS PLAN]` shows requested and
  planned XYZ/RPY and full rotation error. `PickMotionConfig.h` holds physical settings;
  `TrajectoryTestConfig.h` holds the imaginary part wrapper;
  `ClassificationConfig.h` holds the shared sorting layout and ID mapping.
  Each cycle logs its X, pickup Z and approach Z for repeatability testing.
- Y freezes the imaginary part's clock immediately. The active point movement
  or claw action finishes, then the controller pauses before the next action.
  A resumes the sequence. Pausing while waiting freezes the remaining arrival
  countdown; resume continues that countdown rather than closing early. The
  selected X and corrected pickup Z are held for the whole cycle.
- L1 cancels the test and clears pause, suppressing pending pickup timers. It
  recovers to HOME and preserves a closing/closed claw. After settled HOME is
  confirmed it returns to CALIBRATION_MENU. A/Y cannot interrupt that
  cancel-return; the emergency button remains available.
- After successful sorting/opening, the arm returns directly to HOME with the
  claw open, using the same joint/TCP arrival checks, settling time and feedback
  supervision as initial HOME preparation. The test stays active until
  `HomeReady` and then prints `Complete at HOME; A starts a new test`. A during
  return cannot start another cycle. Y/A can pause/resume this return; L1 converts
  it to cancel-return and exits to CALIBRATION_MENU after HOME. Faults still stop
  the movement. This adds a point-to-point movement from each bin to HOME whose
  physical path must be checked on the robot.
- A recovered missed pickup or rejected plan leaves the test ready for another
  A, retaining the existing recovery behavior. Each new test starts with a new
  imaginary part; no tracker calibration changes. Real AUTO also returns HOME
  after a sorted or recovered missed part before selecting another real track.

No camera stream, homography, or belt-speed calibration is used. The test uses
the configured ID-to-bin table and three proposed sorting positions.
Controller connectivity, joint limits, valid configuration, fresh Nucleo
telemetry, stage timeouts, and the emergency latch still apply, including while
paused. A pause keeps the existing target under Nucleo position control.

Y=400 at -100 mm/s gives 4 seconds to Y=0. With the defaults above, approach
starts 1600 ms after release, descent at 3200 ms and closing at 3600 ms; closing
completes no earlier than 4120 ms; transport starts no earlier than 4220 ms with
the extra 100 ms. Each new test independently randomizes X,
mapped ID/bin, and a tall or wide imaginary box (50/50). HOME preparation occurs
before release. Pauses retain that cycle's choices and freeze the imaginary
part clock; real telemetry and already-started gripper action timers stay real.
Before closing starts, Y finishes the active movement before pausing. Once
closing has started, Y waits for closing plus the extra 100 ms, then pauses
without requiring gripping-height tolerance.
No new timed action starts while a pause request is pending. AUTO never freezes
the physical part clock.

Physical validation is required for the new yaw-change descent and overlapping
closing. Desktop IK/feedback simulation cannot establish actuator travel time,
collision clearance or that a real part was captured.

### Shared arrival checks

The supplied waypoint log fluctuated between roughly 4.8 and 5.4 mm of TCP
error, repeatedly resetting the strict 5 mm settling timer. The HOME logs had
q4 errors of -1.18 to -1.42 degrees, outside the previous 1-degree joint limit
even while TCP error was below 5 mm. These are two separate admission failures;
the logs do not establish that sensor readings are wrong or that the true
physical endpoint is exact. At APPROACH/recovery, another supplied log had only
2-3 mm TCP error but q4 alternated between -0.78 and -1.02 degrees. Applying the
1.5-degree allowance only at HOME/waypoint left these stages blocked by the old
1-degree wrist limit, causing a missed pickup followed by a recovery timeout.

AUTO and TRAJECTORY TEST use one editable `wristArrivalToleranceDeg` (1.5 degrees)
at every stage: preparation/final HOME, approach, descent, waiting, closing,
classification waypoint, final drop/open and missed-pick recovery. Other joints
retain their configured limits (1 degree by default). It also enables an
optional TCP arrival band for HOME and the classification waypoint. Settings are
in `PickMotionConfig.h` (these arrival allowances remain build settings):

- `wristArrivalToleranceDeg`: the shared q4 allowance, currently 1.5 degrees.
- `wristHoldingToleranceDeg`: 2.0 degrees, enabled only after the current target
  has met its entry tolerances, full settling time and minimum movement time.
  It is shared by AUTO, TRAJECTORY TEST and joint-motion-test trajectories.
  Each new movement resets this latch: an initial q4 error of 1.51 degrees
  still cannot qualify as arrival. After confirmation, that same reading can
  hold the pose without an immediate SAFETY_HEIGHT_LOST/PICKUP_POSE_LOST.
  Other joints retain their existing limits. TCP checks, feedback freshness,
  deadlines and recovery remain active. A q4 error above 2.0 degrees still
  loses the held pose. These are code settings, not persisted tuning fields.
- `homeWaypointJointToleranceDeg`: {1,1,1,wristArrivalToleranceDeg,1} in
  pitch/roll/j3/q4/q5 order.
- `homeWaypointPositionToleranceMm`: 5 mm, required to begin settling.
- `homeWaypointPositionHysteresisMm`: 1 mm; once settling begins, TCP errors up
  to 6 mm retain the timer. Above 6 mm, or outside any joint limit, the timer
  resets and TCP must enter 5 mm again. Merely starting at 5.4 mm cannot qualify.

The configured settling duration remains 150 ms, using new telemetry sequences
and telemetry timestamps. Repeated use of one packet does not add stable time.
This is a larger acceptance envelope for HOME/waypoint, not a sensor
correction or a guarantee of physical accuracy. It can accept a final TCP error
between 5 and 6 mm after valid entry. During a pause at either stage, the same
hold envelope is monitored and excessive drift still faults. Feedback freshness,
motion timeouts and the emergency latch remain active.

`[ARM]` and `[PICK JOINTS]` show `q4entry/hold=1.50/2.00deg` and
`holdActive=0/1`. The displayed joint tolerance/outside mask uses the active
entry or holding limit. Joint holding hysteresis begins after full confirmation;
the pre-existing HOME/waypoint TCP hysteresis begins when settling starts.

Approach, pickup/closing, final classification/opening and missed-pick recovery
retain a strict 5 mm TCP limit; the 6 mm hold band does not apply there. The q4
allowance applies to their joint checks and paused-pose checks as well. It does
not force descent/closing, remove TCP checks, or move the close deadline (3.7
seconds in the synthetic test with current defaults; real AUTO uses measured Y
and calibrated speed). A pre-close miss leaves the claw open. If closing has
already started, recovery preserves the closed claw and requires intervention. Both AUTO and test apply
the shared profile to their configuration; generic `AutoConfig` defaults alone
retain the legacy 1-degree/5-mm checks for other callers. Invalid arrival settings reject
motion before sending a command. Neither Nucleo control nor FK/IK targets are
adjusted by this policy.

Internally `AutoConfig::jointToleranceOverrideDeg` uses pitch/roll/j3/q4/q5
order; 0 inherits `jointToleranceDeg`, while a positive finite value overrides
that joint's allowance. Both modes set element 3 from the shared wrist constant.
HOME/waypoint use their explicitly configured joint array when that profile is
enabled. Negative or nonfinite overrides reject motion.

`[ARM]` diagnostics now print `entry`, `TCPentry/hold` and `jointTol`.
`inTolerance` shows the active check: the entry band before settling, the hold
band afterward. `entry=0 inTolerance=1` is expected inside the hysteresis band.
Fault diagnostics retain the limits of the stage that failed.

### Diagnosing HOME and recovery timeouts

HOME entry sends `S:AUTO`, the zero-joint `P:` target, and the open `G:` target
as one newline-delimited UART write. Current Nucleo V2 uses circular DMA and a
complete-line FIFO. The batch retains command order and compatibility with its
older single-mailbox receiver; it does not add command acknowledgements to the
Nucleo interface.

AUTO and the test print `[ARM]` progress once per second while active and immediately
on a controller fault. It reports elapsed stage time, telemetry age/validity,
time held inside tolerance, FK TCP position error, the target joints, and signed
joint errors in `pitch,roll,j3,j4,j5` order. A fault keeps its originating stage
for diagnostics instead of printing a misleading transition to IDLE first.
`[ARM TX] HOME batch` confirms the Portenta submitted the initial command batch;
it cannot prove the Nucleo executed it.

Both modes use 150 ms of settled telemetry, the shared wrist allowance and the
HOME/waypoint TCP band described above. Other motion stages retain the 5 mm TCP
check. Small joint errors can exceed the TCP
tolerance on this arm. The 10-second motion timeout remains unchanged.
Fresh telemetry does not prove that
the target has been reached or that each sensor reading is fresh.

APPROACH followed by MISSED-PICK RECOVERY means a scheduled pickup deadline
passed before the pickup pose was ready. Recovery holds/commands the approach
point without closing the claw. If that point also fails to settle within the
motion timeout, the controller latches ESTOP. Review its `[ARM]` errors before
changing tolerances, motor settings, or the physical belt speed.

Runtime pickup timing/height edits are in AUTO PICKUP TUNING. Build geometry defaults
are in `PickGeometryDefaults.h`, arrival settings in `PickMotionConfig.h`, routing in
`ClassificationConfig.h`, and AUTO admission in `AutoPickSelection.h`.
Camera-part X correction is separately editable per WIDE/TALL profile as
**Camera part X offset** in AUTO PICKUP TUNING, in 0.5 mm steps (-100..100 mm). Its build default is
`CameraTracking::partXOffsetMm` in `CameraTrackingConfig.h` (currently -9 mm).
Positive adds to robot X, negative
subtracts: measured X=300 mm with +15 mm becomes X=315 mm. The correction is
applied once to filtered unshifted homography X, before pickup
workspace checks and evaluation of the fitted pickup height. Association uses
unshifted X so ratio changes cannot split a track due to a correction jump. AUTO logs this
offset on entry and assigned-track X is already corrected. Moving-AprilTag
velocity calibration reports unshifted homography X; it does not apply the
part-position correction. A constant X correction does not change Y speed.
April-tag calibration, synthetic trajectory-test coordinates, manual targets,
HOME and classification poses retain their existing coordinates. X applies
the draft at IDLE and clears old tracks; R1 saves after stream shutdown without
reflashing. Homography and belt velocity remain valid. This is a constant empirical correction; it cannot
correct an error that varies with part height or position across the image.
Other timings/tolerances are in `AutoConfig` in `ArmTrajectory.h` or setup overrides.
`openAngle` and `closeAngle` must stay inside the limits received from Nucleo.
For example, `closeAngle=165` with a received maximum of 160 prevents AUTO and
the trajectory test from starting even with valid calibrations. Start rejection now
prints the first specific blocker and the configured angles, received gripper
limits, and feedback status/age. Fixing one blocker may reveal another.
The default timings and tolerances
are initial assumptions requiring robot measurements:

- `closingLeadMs`: close-command to intended capture instant.
- `closingDurationMs`: close-command to completion before transport.
- `openingDurationMs`: open-command to completion before another pick.
- `descentLeadMs`, `pickUpMarginMs`: descent timing and approach margin;
  `pickUpBudgetMs()` is derived, not an independent knob.
- `scheduleToleranceMs`: approach/descent command scheduling jitter allowance.
- `jointToleranceDeg`, `jointToleranceOverrideDeg`, `positionToleranceMm`, `settleMs`: actual positioning
  repeatability and required stable telemetry duration.
- `maximumLateMs`, `maximumCaptureErrorYmm`: permitted capture timing error.

The remaining close-command delay is `-y/vy*1000 - elapsedMs - closingLeadMs`,
where y and its timestamp come from the assigned snapshot and vy is negative
mm/s. A negative delay means the deadline has passed. No negative floating-point
delay is cast into an unsigned absolute deadline.

Camera calibration requires exactly one stationary tag of each ID 0..3 in a
valid frame. Recalibration clears live tracks and preserves the independently
calibrated belt velocity. Remove the
moving calibration tag before CAMERA CALIBRATION, especially because it may
share an ID with a corner tag. ESP32 now forwards all valid tag detections;
Portenta rejects duplicate homography IDs rather than silently taking the first
four detections. Successful calibrations remain in RAM until SAVE ALL in
TUNING & STORAGE, then restore from QSPI Flash on subsequent boots.

### Controller-loss supervision by mode

Controller loss triggers ESTOP in MANUAL_0, MANUAL_1, AUTO with MOVE ARM,
TRAJECTORY TEST, JOINT MOTION TEST and JOINT CALIBRATION. It is enforced for
the whole motion-capable mode, including idle, HOME holding and paused stages;
it does not depend on an instantaneous movement flag.

MODE SELECTOR, AUTO MENU, CALIBRATION MENU, algorithm switching, CAMERA
CALIBRATION, BELT VELOCITY ESTIMATION and AUTO ONLY VISION continue through
controller gaps. Menu selections wait for controller traffic to recover.
Calibration samples, accepted-pass statistics and camera streaming are not
cleared by a controller timeout. Finishing calibration can return to the menu
while the controller is still absent. Entering a motion mode restores the
controller-loss stop requirement.

Both the immediate post-poll check and global supervisor use the same policy.
The UDP timeout value and link validation are unchanged: timeout/recovery
diagnostics and the real link status remain active in every state. Non-motion
timeouts print `no motion-mode ESTOP required` instead of `failsafe activated`.
They also preserve pending/confirmed camera algorithm selection; a controller
gap alone cannot cancel algorithm-switch completion. Actual camera events,
peer changes and transaction failures retain cache invalidation.
Received emergency-button events still latch ESTOP in menus and calibrations;
camera offline and ESP32 transaction failures retain their existing checks.
Link recovery never clears an already latched emergency.

### Moving-AprilTag belt velocity calibration

BELT VELOCITY ESTIMATION selects AprilTag algorithm 16, the same algorithm
used for homography. It collects TEN accepted independent passes before saving
the arithmetic mean of their fitted velocities. Each pass has equal weight;
passes with more samples do not dominate the session. Pass ONE small AprilTag flat on the running belt, using
physical ID 0, 1, 2 or 3. The four stationary homography tags can remain visible:
selection is by image geometry before ID, so a corner tag with the same ID as
the moving tag is ignored when outside the velocity rectangle.

Edit `CameraTracking::VelocityRegion` in `CameraTrackingConfig.h`:

```cpp
constexpr PixelRectangle rectangle = {224, 48, 416, 432};
constexpr uint8_t minimumTagId = 0, maximumTagId = 3;
```

The rectangle is in the 640x480 image's pixel coordinates, before homography:
X=224..416, Y=48..432. It is taller than it is wide. Every edge of the moving
box must fit inside it, and all four fixed tags must stay outside it. The moving
tag should lie in the same physical plane used for the homography reference.
Moving or
rotating the camera requires homography again even if the tags stay fixed on
the belt. Tag IDs still map to the original robot-world X/Y reference; do not
swap world axes because the part moves vertically on screen.

The first accepted in-region tag locks the calibration ID. Empty/corner-only
frames and missed detections do not refresh its accepted-sample timestamp or
alter its regression. A different eligible ID in the lane rejects the run;
two eligible tags inside the rectangle also reject it, even if their IDs are
the same. Use one moving tag at a time. Unconfigured IDs inside the rectangle
are ignored and counted. The tag centre is projected with the homography;
`partXOffsetMm` is not applied to this calibration target.

Successive camera polls can repeat an unchanged detection while inference is
still processing. Consecutive identical pixel centres are counted once, even
when the reported box dimensions change. `repeats` counts ignored repeats;
they do not extend the fit span or refresh the last accepted-sample time. This
avoids fitting a staircase of held results as if the tag repeatedly stopped and
jumped forward. A changed position first received within the 80ms sample gate
is discarded, rather than accepted later with a delayed timestamp. A subsequent
changed position may be accepted normally. The four-sample requirement means
four accepted coordinate changes, including the initial position.

This is coordinate deduplication, not a camera frame-ID check. Position jitter
can still appear as new data; acquisition latency/jitter and perspective errors
remain possible. Repeat physical passes to validate the resulting speeds.

Small-tag detection may be intermittent. `BeltCalibrationLimits` in
`BeltTracker.h` now requires FOUR accepted samples, at least 800 ms of observed
span, at least 20 mm travel along negative world Y, and negative fitted speed
with RMS residual <=8 mm. Samples closer than 80 ms are ignored. The existing
X-step (20 mm), reverse-Y-step (10 mm), and accepted-sample gap (1500 ms) checks
remain. After 1500 ms without an accepted position, the current pass is
evaluated. A pass is also evaluated after 20 seconds from its first accepted
position. A stationary corner tag cannot
pass the travel/direction gates merely by being detected four times.

The session runs as follows, without moving the arm:

1. Keep the moving tag outside the measurement rectangle. The Portenta must
   receive 1500 ms of clear-lane frames, with no gap over 500 ms between them.
   A configured tag even partly overlapping the rectangle blocks rearming;
   fixed corner tags must be completely outside it. No incoming frames do not
   establish that the lane is clear.
2. Wait for `[CAL] PLACE TAG: run 1/10`, then place the flat tag on the running
   belt. Let it pass through the rectangle and leave it. Missed detections do
   not count as new measurements.
3. A passing fit prints `Pass ACCEPTED` and increments `accepted`. A failed fit
   prints its reason, increments `rejected`, and requires a retry of the same
   run number. Both outcomes require a clear lane and another `PLACE TAG`
   prompt before the next insertion. The camera stream stays active between
   passes; no algorithm switch or stream-start delay is repeated.
4. After ten accepted passes, `[CAL] COMPLETE` reports the mean vy, sample
   standard deviation across passes, minimum/maximum signed vy and rejected
   count. The mean becomes AUTO's calibrated velocity only at this point. The
   camera stream is stopped and the calibration menu returns.

There is no operator-wait timeout before a tag is seen or between passes.
L1 cancels, discards the partial session and ends the stream. Starting a session
invalidates any old belt velocity, so cancellation or an interrupted session
leaves AUTO unavailable until a complete calibration succeeds. Controller-link
diagnostics, camera faults and received emergency presses remain supervised;
controller loss alone does not interrupt this non-motion calibration. A pending
START acknowledgement is completed before a cancellation sends END.

Edit `BeltCalibrationSessionConfig` in `BeltCalibrationSession.h` for the number
of accepted passes (at least ten), clear-lane duration, allowed clear-frame gap
and per-pass timeout. Keep the belt speed unchanged throughout the session.
The operator must recalibrate if actual belt speed changes, including changes
after belt power-on; AUTO cannot detect them. The mean can be persisted with
SAVE ALL in TUNING & STORAGE. Power-on does not automatically invalidate it. Averaging reduces random
between-pass scatter but does not correct homography or camera timing bias.

Calibration prints the ROI and requirements on entry. `[CAL SAMPLE]`,
`[CAL RESULT]` and `[CAL REJECTED]` include sample count, span, travel, deltaX,
fitted vy, RMS, last projected pixel/world position, `outsideROI`, `ignoredIDs`,
`repeats`, `tagLocked`, and `tagID`. Additional rejection reasons include
`MULTIPLE_TAGS_INSIDE_ROI`, `MOVING_TAG_ID_CHANGED` and `INVALID_TAG_FRAME`.
No partial session or failed fit becomes the tracker's calibrated velocity. Four samples
are a minimum for this trial, not a guarantee of physical velocity accuracy.
RMS acceptance remains the existing operator-approved 8 mm trial limit.

ESP32's AprilTag stream no longer requires all four corner IDs to be recognized
or truncates the frame to four boxes. It forwards valid tag content IDs, including
duplicates, up to the existing ten-box capacity. Non-numeric/out-of-range tag
content is discarded rather than converted to ID 0. Successful camera reads
with zero detections also produce empty frames, allowing sparse observations
without mistaking absence of tags for camera link failure. The VisionPacket
layout is unchanged. Both ESP32 and Portenta sketches must be uploaded.

### AUTO uses a fixed calibrated speed

Per-part velocity estimation, rolling speed histories, and the automatic
"Belt velocity changed" fault have been removed. AUTO uses ONLY the stored
AprilTag-calibrated belt velocity for prediction and pickup timing. Camera
positions still update track geometry and class/ID association over the full
image, independently of the arm's assigned snapshot. Different part shapes,
new parts during sorting, or a mismatch with apparent box speed do not overwrite
that speed or trigger a belt-speed emergency.

**The robot cannot detect an actual conveyor speed change.** AUTO menu/entry,
velocity-calibration entry and successful calibration explicitly remind the
operator: stop AUTO, recalibrate after EVERY speed change, then restart AUTO.
Controller loss, camera loss, stale arm feedback and the other existing motion
faults still apply. Position-only vision does not provide independent belt-speed
verification either.

A flat moving tag removes part-box deformation/height from the speed measurement;
it does not correct part-position parallax during later AUTO pickups. Current
vision timestamps remain Portenta receive times, so transport/acquisition timing
uncertainty remains. Repeat tag passes at unchanged belt speed and compare the
results before relying on physical pickup timing.

### Reusing the selected camera algorithm

`CameraAlgorithmSelection.h` records the last completed switch in Portenta RAM.
A matching command ACK and a completion event are both required. Selecting an
operation using that same algorithm prints `[CAM] Algorithm ... already
confirmed; reusing` and enters the operation without another switch request or
warmup. The operation still performs its normal start/end-stream handshake.
Homography and velocity calibration both use April tags and can reuse each
other's confirmed selection. Changing between April tags, model 131 and model 129 retains the existing switch
procedure and its 12-second deadline.

Camera online/offline events, controller-link timeout/peer change, critical
transaction failure, unavailable algorithm and switch timeout invalidate the
record. A Portenta restart also starts unknown. Consequently the first operation
after those events performs a full switch. This records the existing ESP32
completion exchange; it does not add camera algorithm readback or detect an
operator changing the algorithm directly on the camera screen.

## Remaining hardware/interface limits

Nucleo reports A packets nominally every 50 ms. The 300 ms timeout detects absent
packets, not independent freshness of every sensor in a received packet. The
existing firmware can retain an old wrist sensor reading after a failed read;
PWM wrist/gripper telemetry is command-derived. Grasp completion therefore uses
a calibrated time assumption and cannot prove a part was captured.

Vision timestamps are Portenta receive times; the current packet has no camera
acquisition timestamp or frame sequence. Transport delay and reordered frames
cannot be fully corrected by the Portenta alone. ESP32 acknowledgements/events
are checked using the existing protocol and do not independently verify camera
algorithm readback.

A serial ESTOP depends on the Portenta and link remaining operational. An
independent physical emergency-stop/watchdog mechanism is required for failure
of the master or communication link. This implementation is a supervised
development controller, not a certified machine safety system.

Adding the TCP offset does not by itself explain the previous belt-contact Z
variation. Record settled joint angles, reported TCP orientation, and independent
physical contact coordinates before fitting a belt plane or changing DH geometry.

## Build

### UDP startup regression

The installed Mbed Portenta 4.6.0 UDP implementation reports the original
datagram length from `available()` even after `read()`. Draining a rejected
packet with `while (available()) read()` can therefore freeze the loop. The
vision receiver now rejects the packet and advances with the next bounded
`parsePacket()` call. Vision now uses `NonBlockingVisionUdp.h`, a project-local
Mbed socket adapter whose RX/TX remain nonblocking on the same 4220 port with
the same VisionPacket format. This also avoids Arduino MbedUDP `endPacket()`
switching vision TX to a 1000 ms blocking timeout, longer than the controller's
200 ms watchdog. Unsent vision messages are counted; existing critical-message
and event-ACK retries handle retransmission. Board-package files are unchanged.

Controller RX remains on Arduino's socket, but services up to eight datagrams
or two milliseconds per poll before evaluating the unchanged 200 ms watchdog.
It is polled before and after serial/vision work. A malformed, duplicate or
late first datagram no longer hides a newer valid packet behind it in that
bounded batch. Invalid packets never refresh the watchdog. Button rising edges
and any emergency press are retained across the batch even if the final packet
shows release. A timeout detected earlier in the iteration remains an emergency
request even if a later poll receives a packet; recovery never resumes motion.

On a timeout, two `[LINK]` diagnostic lines report accepted-packet age,
controller polling gaps, maximum vision-work duration, sequence number,
received/accepted counts and rejection/socket/send-failure counters. Counts and
maxima are cumulative since boot. Large polling gaps suggest delayed Portenta
service; small polling gaps with no accepted traffic indicate a gap in delivery
or validation. Increasing rejection counters identifies why received datagrams
were not accepted. These observations do not by themselves distinguish RF,
ESP32 scheduling, driver delays or electrical interference. A steady ESP32 LED
does not establish arrival of fresh validated controller datagrams.

The timeout line also includes cumulative `missingSeq` and `maxRxGap`. The first
accepted packet after a timeout produces one `Traffic resumed` line after
supervision, containing the complete receive gap and its sequence step from the
last accepted packet before the timeout. The ESP32 increments its sequence when
it attempts a controller send. With `comparable=1`, a step greater than 1 means
intermediate sequence numbers were not accepted here (send failures, transport
loss/delay or rejected datagrams are possible); step 1 suggests a sender pause
or a delayed next datagram. This is evidence, not a unique diagnosis. Peer
changes and non-forward counters are marked incomparable; sequence and clock
wrap are supported. Recovery printing is outside the receive-draining loop and
never clears the emergency latch. The 200 ms watchdog is unchanged.

A validated controller packet registers the ESP32 peer immediately, before
vision processing in that same loop. Full reads and checksums are checked before
registration. Reconnection can update the peer address when the controller link
is down; packets from a different peer cannot replace an active link.

Serial output now includes `[LINK] Controller peer: ...` and positive button
edges (`[DEBUG] Presionado: ...`). State-entry messages indicate that the loop
continues. Nucleo pose printing in manual modes still depends on changed angles.

`python tests/udp/run_tests.py` exercises the actual sketch receive functions and
the installed `MbedUdp.cpp`, with only its socket transport mocked. It covers
camera-first and same-loop startup, malformed packets, peer changes, bounded
processing, and ACK/frame validation. The same emulator dependencies as the
trajectory tests are used. `--reproduce-old-drain` deliberately restores the old
discard loop in a temporary test build and confirms that execution hangs; it
never modifies or flashes the sketch. Use `--udp-source` for a different board
package path. These tests cannot establish actual radio/link performance.

The controller regression harness in `tests/trajectory` runs the real ARM C++
trajectory and IK implementations in an emulator, with commands/feedback
simulated. It covers normal completion, finishing a movement before pausing,
frozen arrival timing, pauses during closing and HOME, cancel-return, stale or
changed feedback while paused, clock wrap, and configured admission budgets.
It also extracts the actual sketch HOME adapter to verify one ordered UART
write, failed writes/limits/stale feedback, preserved fault-stage diagnostics,
slow-approach recovery, and TCP tolerance failures despite small joint errors.
IK coverage includes 100 reachable-pose rounding regressions, independent XYZ
and orientation increments at HOME and a bent pose, strict versus relaxed
orientation, rejected-target rollback, the supplied measured HOME seed, and
separate deadline/approach/classification rejection diagnostics.
The same harness extracts the actual AUTO coordinator and global supervisor
from the sketch. It exercises real tracker admission and ID routing, shared
geometry equivalence across 39 X/bin combinations, three full camera-fed cycles
at different calibrated velocities, HOME return, normal Stop/exit, simultaneous
Stop/A, real-clock tracking during return, ONLY VISION with absent arm feedback,
ignored/outside/late tracks, startup cancellation, stale feedback and emergency
latching. Transport acknowledgements and physical motor feedback are simulated.
It also checks sparse moving-tag calibration, duplicate corner IDs, whole-box
ROI selection, invalid tag frames/content, tag-ID continuity, the real calibration
state, and that apparent part speeds never change the stored belt velocity.
Arduino does not compile this test-only subdirectory into the sketch.

To run it with Python, install `unicorn` and `pyelftools` (either normally or into
`%TEMP%/codex-portenta-test-dependencies`), then run
`python tests/trajectory/run_tests.py`. The script accepts `--compiler` and
`--bla` paths when they differ from this machine's defaults. Its executable
output is written under `%TEMP%/codex-portenta-trajectory-tests`; it never
connects to or flashes a board. Simulation cannot establish physical path safety
or actuator timing.

Built for Portenta H7 main M7 core, 50/50 flash split, using Arduino Mbed Portenta
4.6.0 and BasicLinearAlgebra 5.1. A compile does not flash the board or establish
hardware motion/capture correctness.

```powershell
& 'C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe' compile --fqbn 'arduino:mbed_portenta:envie_m7:target_core=cm7,split=50_50,security=none' --libraries 'C:\Users\diego\Documents\Arduino\libraries' --warnings all 'C:\Users\diego\OneDrive\UVG\PryGrd\Code\Code_V3\Portenta_Core_V3'
```
