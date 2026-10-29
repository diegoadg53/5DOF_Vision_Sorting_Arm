# Controller tuning and persistent calibrations

Upload the Portenta sketch once to install this update. No Nucleo or ESP32
update is required for pickup profiles. Subsequent changes use the controller
and Serial Monitor. A normal internal-Flash firmware upload retains these QSPI
records; an external-QSPI erase or reformat removes them.

## AUTO PICKUP TUNING

From CALIBRATION MENU select **AUTO PICKUP TUNING**, then X. This mode uses live
camera parts, **ID IGNORANT**, **model 131**, and **MOVE ARM**, without changing
your normal AUTO_MENU selections. It needs camera connection to enter and valid
homography, belt velocity, joint limits and feedback to start moving.

- **A** starts streaming and supervised HOME/open preparation, then AUTO picking.
- **Y** finishes the assigned cycle and HOME return, then stops streaming.
- **L1** finishes/HOME and exits to CALIBRATION MENU.
- **B**, controller loss and active-motion feedback faults retain the existing
  emergency behavior of AUTO. This is a motion-capable mode, including its IDLE
  periods; it does not inherit the controller-loss exemption of passive menus.
- **R3** finishes/HOME and opens the general TUNING & STORAGE menu.
- **L3 alone**, while stopped, resets all six AUTO counters. Use Y to finish/HOME
  and stop first. Return the stick to neutral before editing again. Counter
  reset does not modify tuning or saved calibrations.

Editing is available only while the arm is **IDLE**, whether stopped or waiting
for a part. It is locked throughout planning, scheduled waiting, movement,
sorting, recovery and the final HOME return. A stick held through a cycle must
return to neutral before editing resumes. Button presses during the cycle are
not queued for later application.

At IDLE:

| Control | Action |
| --- | --- |
| Left stick up/down | Select a setting: nine WIDE rows, ten TALL rows |
| Left stick left/right | Adjust the selected draft value |
| L2 | Switch the displayed WIDE/TALL draft profile |
| X | Validate and apply **both** draft profiles to RAM |
| R1 | Stop camera streaming, apply and save both profiles plus calibrations |

Stick repeat starts after 500 ms, then repeats every 200 ms. While manipulating
the editor, no new part is assigned in that loop. After an R1 save the mode stays
**stopped**; A starts it again. Flash writes occur with no active trajectory or
pending camera transaction, after sending Nucleo `S:MODES`. A save failure leaves
the applied RAM values available and prints the storage reason. Changes are
never saved automatically on each adjustment.

## The two pickup profiles

- **WIDE:** last accepted box has `height <= width`; pickup yaw **0 degrees**.
- **TALL:** last accepted box has `height > width`; pickup yaw **90 degrees**.

WIDE and TALL complete pickup **closingDuration + 100 ms after the actual close command**,
using each profile's own closing duration and without requiring gripping-height
position/settling confirmation or confirmation by predicted Y=0. Both use position checks at
safety height, classification and HOME, and retain feedback/emergency supervision.
The extra shared delay is the code setting `PickMotion::pickupPostCloseMs` in
`PickMotionConfig.h`, not a saved tuning row. Existing Flash tuning/calibrations
remain compatible. `[PICK POLICY]` and `WIDE_TIMED_COMPLETE` / `TALL_TIMED_COMPLETE`
identify this policy and the locked pickup profile.

Each profile independently contains:

| Setting | Step | Validation |
| --- | --- | --- |
| Approach margin | 10 ms | Positive; controls approach start budget |
| Descent lead | 10 ms | Positive and >= closing lead |
| Closing lead | 10 ms | >=0 and <= descent lead |
| Closing duration | 10 ms | Positive and >= closing lead |
| Opening duration | 10 ms | Positive |
| Approach TCP Z | 0.5 mm | -100..500; above fitted pickup Z over the whole supported X range |
| Pickup height adjustment | 0.5 mm | -50..50; negative lowers TCP |
| Reference contact Z | 0.5 mm | -150..150 |
| Camera part X offset | 0.5 mm | -100..100; added to transformed camera X |
| ID 1 TALL extra X (TALL only) | 0.5 mm | -100..100; default 0; applied only in ID-classification AUTO |

The original contact-Z value can include fractional millimetres (-40.104 by
default). Each edit adds/subtracts 0.5 mm, preserving that fractional base. Time
values must satisfy the controller's timing bounds as well as the editor range.

The fixed slope, gripper angles, bin poses, joint limits, arrival tolerances and
tracking rectangle remain code settings. Height reference X is **shared** and
stays in the general menu. For either profile:

```text
pickupX = assignedUnshiftedCameraX + selectedProfile.partXOffsetMm + idExtraX
idExtraX = TALL.id1ExtraXmm only for stable ID 1 + TALL + ID-classification AUTO
           0 for other IDs, WIDE, ID IGNORANT and synthetic tests
pickupZ = selectedProfile.contactZ + selectedProfile.heightAdjustment
          + fixedSlope * (pickupX - sharedReferenceX)
pickupBudget = approachMargin + max(descentLead, closingLead)
```

The positive X correction moves pickup toward larger robot X. It does not shift
HOME, manual targets, bins, AprilTag calibration, or synthetic test X. Applying
a changed offset clears tracks and waits for fresh frames; H and belt speed
remain valid. Track association uses unshifted camera X so a box-ratio change
does not create a second track merely because the two offsets differ.

### Extra correction for ID 1 in TALL orientation

The extra is **added to** the normal TALL camera offset. Positive moves the
pickup toward larger base X; negative moves it toward smaller X. It defaults
to 0 mm and is editable in 0.5 mm steps. It changes final pickup X, its fitted Z,
the planned safety/pickup targets and workspace admission. Association continues
to use the original camera coordinates, so this correction cannot create tracks.

To tune it while testing ID-classification AUTO, press **R3** to finish/HOME and
open **TUNING & STORAGE**, select **ID 1 TALL extra X**, edit with the left stick,
then A to apply. Use SAVE ALL TO FLASH + X to persist. L1 returns to AUTO; A
restarts it. The same value is the tenth TALL row in AUTO PICKUP TUNING; X applies
and R1 saves there. These are two views of one stored value.

Only ID-classification AUTO applies this extra, using the confirmed ID frozen at
assignment. ID IGNORANT, ONLY VISION and AUTO PICKUP TUNING retain their existing
positions/behavior; their logs/tuning row identify the restriction. Thus test the
physical correction in ID-classification AUTO, not the ID-ignorant tuning run.
TRAJECTORY TEST and JOINT MOTION TEST keep their synthetic X unchanged.

Both profile variants are preplanned with that confirmed identity: ID 1 may
switch WIDE/TALL before descent and then use the corresponding corrected X.
Changing the box later does not change the assigned identity or bin. Nonzero
corrections that put the final selected X outside the tested pickup range are
rejected; the code does not extrapolate the belt-height fit. `[AUTO] Assigned`
reports the starting profile, corrected X and applied `idExtraX`.

Both profiles' safety, pickup and classification joint targets are preplanned
incrementally before movement. A cycle owns a frozen copy of both configurations.
The **selected profile stays live until descent**, following the assigned
track's last accepted box. Its class/bin, unshifted assignment X, Y/time snapshot
and calibrated belt velocity are not replaced by later detections.

Changing ratio before approach changes its scheduled budget and selected safety
target. Changing ratio during approach or the safety-height wait can change X
and Z: the arm commands the newly planned safety pose and must settle there
again. If the new profile is unavailable or cannot meet its descent deadline,
the pickup is recoverably missed and returns HOME. At descent, profile and yaw
lock together, including closing/opening times. No later observation changes
that cycle. `[PICK PROFILE]` reports selection and active settings; the existing
`[PICK TIMING]` / `[PICK MISSED]` diagnostics report deadline/arrival behavior.

TRAJECTORY TEST selects the same WIDE/TALL configurations from its random/fixed
imaginary box, with no camera X correction. JOINT MOTION TEST uses TALL for its
fixed tall imaginary part and retains its repeated HOME measurements.

## General TUNING & STORAGE

Select it directly in CALIBRATION MENU, or press R3 from AUTO/TRAJECTORY TEST
and finish the current cycle/HOME. A paused test needs A to resume first.
No camera or calibration is needed to open this passive menu.

The six editable rows are:

1. Shared reference X (0.5 mm).
2. Test X: random/fixed.
3. Test fixed X (0.5 mm, inside the existing pickup range).
4. Test yaw: random / 0 / 90 degrees.
5. Test bin: random / 1 / 2 / 3, requiring an enabled mapped route.
6. ID 1 TALL extra X (0.5 mm; applied only in ID-classification AUTO).

The four action rows are APPLY TO RAM, SAVE ALL TO FLASH, RELOAD SAVED SETTINGS
AND CALIBRATIONS, and BUILD DEFAULT SHARED/TEST SETTINGS. Left stick navigates
and edits; A applies the draft; X performs the selected action; L1 discards
unapplied changes and returns. Applied RAM changes remain. Build defaults here
reset only shared/test settings in the draft, retaining both pickup profiles
and calibrations. Reload restores the whole saved record, including profiles.
SAVE ALL also preserves both profiles even though they are edited in the new mode.

This passive menu commands `S:MODES`, performs no arm/gripper movement and does
not ESTOP on controller loss. Controls wait for fresh controller packets.

## Camera and belt calibration storage

Run CAMERA CALIBRATION and/or ten-pass AprilTag BELT VELOCITY ESTIMATION, then
explicitly SAVE ALL in the general menu or use R1 at IDLE in AUTO PICKUP TUNING.
Confirm the verified-save message and a new storage generation number.

New/partial calibrations are not automatically saved. Starting a new belt-speed
session invalidates RAM velocity until all ten passes succeed; cancellation does
not restore it silently. Failed/cancelled homography leaves RAM H invalid.
Saving while calibration is invalid stores that invalid flag. The previous
Flash record remains available until another explicit save replaces it.

A homography recalibration clears tracks but **retains calibrated belt speed**.
The operator must recalibrate H whenever camera/reference geometry changes, and
speed whenever actual conveyor speed changes, including after belt power-on.
AUTO cannot detect either change. Boot restores the newest valid record's
profiles, shared/test settings, H and signed mm/s independently of their flags.

## Flash format and compatibility

`PortentaSettingsFlash.h` retains the same checked two-sector QSPI allocation
starting at 14 MiB. It verifies size/alignment, MBR, partition bounds and sector
ownership. It never formats partitions or erases WiFi firmware, certificates,
OTA storage or an existing filesystem. Unrecognized data/layout is refused;
RAM tuning remains available. Do not run QSPIFormat to remove a storage warning.

Schema **4** uses the same 256-byte record, CRC32 and a commit word programmed last.
Each save writes/verifies only the inactive bank, retaining the preceding valid
record if interrupted. Old **schema 1, 2 and 3** records still load without erasing
tuning or calibrations, with the new ID 1 TALL extra set to 0 mm. Schema 1/2's
single pickup configuration initializes **both** WIDE and TALL. Schema 1 uses the
build's camera X offset because it never stored one. Schema 3 retains its distinct
WIDE/TALL settings. The next explicit save migrates to schema 4 in the inactive
bank. Older firmware cannot read schema 4; it may load a remaining legacy bank instead.

Desktop regression checks cover record/commit interruptions, CRC errors, legacy
migration, independent profiles, controller editing locks, live profile changes
and stopped saving. Physical QSPI layout/write/readback and the newly distinct
approach/pickup poses still require verification on the robot.
